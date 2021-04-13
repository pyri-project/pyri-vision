from pyri.plugins.device_type_adapter import PyriDeviceTypeAdapterExtendedState, \
    PyriDeviceTypeAdapter, PyriDeviceTypeAdapterPluginFactory
from typing import List, Dict, Any, NamedTuple
import RobotRaconteur as RR

class Camera_TypeAdapter(PyriDeviceTypeAdapter):
    """Adapter for com.robotraconteur.imaging.Camera"""

    def __init__(self, client_subscription, node):
        self._sub: "RobotRaconteur.ServiceSubscription" = client_subscription
        self._state_sub = self._sub.SubscribeWire("camera_state")
        self._state_sub.InValueLifespan = 0.5
        self._node = node
        self._camera_consts = None

    async def get_extended_device_infos(self, timeout) -> Dict[str,RR.VarValue]:

        res, c = self._sub.TryGetDefaultClientWait(timeout)
        if not res:
            return dict()
        
        info = await c.async_get_camera_info(None,timeout)
        return {"com.robotraconteur.imaging.camerainfo.CameraInfo": RR.VarValue(info,"com.robotraconteur.imaging.camerainfo.CameraInfo")}

    async def get_extended_device_states(self, timeout) -> Dict[str,PyriDeviceTypeAdapterExtendedState]:
        return {}


class PyriVisionTypeAdapterPluginFactory(PyriDeviceTypeAdapterPluginFactory):
    
    def get_plugin_name(self):
        return "pyri-vision"

    def get_robotraconteur_types(self) -> List[str]:
        return ["com.robotraconteur.imaging.Camera"]

    def create_device_type_adapter(self, robotraconteur_type: str, client_subscription: Any, node) -> PyriDeviceTypeAdapter:
        if robotraconteur_type == "com.robotraconteur.imaging.Camera":
            return Camera_TypeAdapter(client_subscription, node)
        
        assert False, "Invalid robotraconteur_type device type adapter requested"

def get_device_type_adapter_factory():
    return PyriVisionTypeAdapterPluginFactory()