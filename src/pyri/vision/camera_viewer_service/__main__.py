import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy as np
import argparse
import RobotRaconteurCompanion as RRC
from pyri.device_manager_client import DeviceManagerClient
import importlib.resources as resources
from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil
from RobotRaconteurCompanion.Util.RobotUtil import RobotUtil
from RobotRaconteurCompanion.Util.ImageUtil import ImageUtil

import time
import threading
import traceback

from pyri.util.service_setup import PyriServiceNodeSetup

class CameraViewer_impl:
    def __init__(self, parent, camera_sub):
        self._lock = threading.Lock()
        self._current_img = None
        self._camera_sub = camera_sub
        self._camera_sub.ClientConnected += self._client_connected
        self._parent = parent
        self._image_util = ImageUtil()
        
        try:
            c = self._camera_sub.GetDefaultClient()
            c.start_streaming()
        except:
            traceback.print_exc()
            pass

        self._camera_sub_preview_sub = None

    def _client_connected(self, sub, subscription_id, c):
        try:
            c.start_streaming()
        except:
            pass

    def RRServiceObjectInit(self, ctx, service_path):
        self.service_path = service_path
        self._downsampler = RR.BroadcastDownsampler(ctx)
        self._downsampler.AddPipeBroadcaster(self.preview_stream)

        self._camera_sub_preview_sub = self._camera_sub.SubscribePipe("preview_stream")
        self._camera_sub_preview_sub.PipePacketReceived += self._preview_new_packets

    def _preview_new_packets(self, sub):
        img = None
        with self._lock:
            while sub.Available > 0:
                img = sub.ReceivePacket()
            if img is None:
                return
            
            self._current_img = img
            self.preview_stream.AsyncSendPacket(img,lambda: None)

    def capture_frame_preview(self):
        with self._lock:
            if self._current_img is not None:
                return self._current_img                 
        
        for i in range(5):
            time.sleep(0.005)
            with self._lock:
                if self._current_img is not None:
                    return self._current_img                
        raise RR.InvalidOperationException("No image available")

    def save_to_globals(self, global_name, save_system_state):
        assert len(global_name) > 0
        var_storage = self._parent.device_manager.get_device_client("variable_storage",0.1)

        var_consts = var_storage.RRGetNode().GetConstants('tech.pyri.variable_storage', var_storage)
        variable_persistence = var_consts["VariablePersistence"]
        variable_protection_level = var_consts["VariableProtectionLevel"]

        if len(var_storage.filter_variables("globals",global_name,[])) > 0:
            raise RR.InvalidOperationException(f"Global {global_name} already exists")
        if save_system_state:
            if len(var_storage.filter_variables("globals",f"{global_name}_system_state",[])) > 0:
                raise RR.InvalidOperationException(f"Global {global_name}_system_state already exists")

        img1 = self._camera_sub.GetDefaultClient().capture_frame()
        img2 = self._image_util.image_to_array(img1)
        img = self._image_util.array_to_compressed_image_png(img2)

        attrs = {}

        if save_system_state:
            attrs["system_state"] = f"{global_name}_system_state"
            devices_states_c = self._parent.device_manager.get_device_client("devices_states",0.1)
            dev_states, _ = devices_states_c.devices_states.PeekInValue()            
            var_storage.add_variable2("globals",f"{global_name}_system_state","tech.pyri.devices_states.PyriDevicesStates", \
                RR.VarValue(dev_states,"tech.pyri.devices_states.PyriDevicesStates"), ["system_state"], attrs, variable_persistence["const"], 
                None, variable_protection_level["read_write"], \
                [], "Captured system state", False)

        var_storage.add_variable2("globals",global_name,"com.robotraconteur.image.CompressedImage", \
            RR.VarValue(img,"com.robotraconteur.image.CompressedImage"), ["image"], attrs, variable_persistence["const"], 
            None, variable_protection_level["read_write"], \
            [], "Captured image", False)

    def save_sequence_to_globals(self, global_name, save_system_state):
        return SaveSequenceToGlobalsGenerator(self, self._parent.device_manager, global_name, save_system_state)

class SaveSequenceToGlobalsGenerator:
    def __init__(self, camera_viewer, device_manager, global_name, save_system_state):        
        self.device_manager = device_manager
        self.global_name = global_name
        self.save_system_state = save_system_state
        self.camera_viewer = camera_viewer

        self.var_storage_c = device_manager.get_device_client("variable_storage",0.1)

        if len(self.var_storage_c.filter_variables("globals",global_name,[])) > 0:
            raise RR.InvalidOperationException(f"Global {global_name} already exists")
        if save_system_state:
            if len(self.var_storage_c.filter_variables("globals",f"{global_name}_system_state",[])) > 0:
                raise RR.InvalidOperationException(f"Global {global_name}_system_state already exists")

        self.image_names = []        

    def Next(self):
        img_name = f"{self.global_name}_{len(self.image_names)}"
        self.camera_viewer.save_to_globals(img_name,self.save_system_state)
        self.image_names.append(img_name)

    def Abort(self):
        # Do nothing, let generator be destroyed
        pass

    def Close(self):
        
        var_consts = self.var_storage_c.RRGetNode().GetConstants('tech.pyri.variable_storage', self.var_storage_c)
        variable_persistence = var_consts["VariablePersistence"]
        variable_protection_level = var_consts["VariableProtectionLevel"]

        # TODO: use list instead of newlines
        self.var_storage_c.add_variable2("globals",self.global_name,"string", \
            RR.VarValue("\n".join(self.image_names),"string"), ["image_sequence"], {}, variable_persistence["const"], 
            None, variable_protection_level["read_write"], \
            [], "Captured image sequence", False)




    

class CameraViewerService_impl:
    def __init__(self, device_manager, device_info = None, node : RR.RobotRaconteurNode = None):
        if node is None:
            self._node = RR.RobotRaconteurNode.s
        else:
            self._node = node
        self.device_info = device_info

        self._lock = threading.Lock()

        self._viewers={}
        
        self.service_path = None
        self.ctx = None

        self.device_manager = device_manager
        self.device_manager.connect_device_type("tech.pyri.variable_storage.VariableStorage")
        self.device_manager.connect_device_type("tech.pyri.devices_states.PyriDevicesStatesService")
        self.device_manager.connect_device_type("com.robotraconteur.imaging.Camera")
        self.device_manager.device_added += self._device_added
        self.device_manager.device_removed += self._device_removed
        self.device_manager.refresh_devices(5)

    def RRServiceObjectInit(self, ctx, service_path):
        self.service_path = service_path
        self.ctx = ctx

    def get_camera_viewer(self, camera_name):        
        with self._lock:
            viewer = CameraViewer_impl(self, self.device_manager.get_device_subscription(camera_name))        
            self._viewers[camera_name] = viewer
            return viewer, "tech.pyri.vision.viewer.CameraViewer"
    
    def _device_added(self, local_device_name):
       pass 

    def _device_removed(self, local_device_name):
        with self._lock:
            if local_device_name in self._viewers:
                service_path = self._viewers[local_device_name].service_path
                del self._viewers[local_device_name]
                try:
                    self.ctx.ReleaseServicePath(service_path)
                except:
                    pass

def main():
    
    with PyriServiceNodeSetup("tech.pyri.vision.camera_viewer", 55916, \
        extra_service_defs = [(__package__,'tech.pyri.vision.viewer.robdef')], \
        default_info = (__package__,"pyri_vision_camera_viewer_service_default_info.yml"), \
        display_description="PyRI Camera Viewer Service", device_manager_autoconnect=False, \
        distribution_name="pyri-vision") as service_node_setup:

        # create object
        CameraViewerService_inst = CameraViewerService_impl(service_node_setup.device_manager, device_info=service_node_setup.device_info_struct, node = RRN)
        # register service with service name "robotics_jog", type "tech.pyri.robotics.jog.RoboticsJogService", actual object: RoboticsJogService_inst
        service_node_setup.register_service("camera_viewer","tech.pyri.vision.viewer.CameraViewerService", CameraViewerService_inst)

        service_node_setup.wait_exit()

if __name__ == '__main__':
    main()