from pyri.plugins.blockly import PyriBlocklyPluginFactory, PyriBlocklyBlock, PyriBlocklyCategory, \
    add_blockly_block, add_blockly_category
from . import sandbox_functions
from typing import List, Dict, NamedTuple, TYPE_CHECKING

def _get_blocks() -> Dict[str,PyriBlocklyBlock]:
    blocks = {}

    add_blockly_block(blocks,
        category = "Vision",
        blockly_json = {
                "type": "vision_detect_aruco",
                "message0": "detect aruco markers %1 marker id %2 marker size (m) %3 ROI %4 %5",
                "args0": [
                    {
                    "type": "field_dropdown",
                    "name": "ARUCO_DICT",
                    "options": [["DICT_4X4_100","DICT_4X4_100"],["DICT_4X4_1000","DICT_4X4_1000"],
                    ["DICT_4X4_250","DICT_4X4_250"],["DICT_4X4_50","DICT_4X4_50"],["DICT_5X5_100","DICT_5X5_100"],
                    ["DICT_5X5_1000","DICT_5X5_1000"],["DICT_5X5_250","DICT_5X5_250"],["DICT_5X5_50","DICT_5X5_50"],
                    ["DICT_6X6_100","DICT_6X6_100"],["DICT_6X6_1000","DICT_6X6_1000"],["DICT_6X6_250","DICT_6X6_250"],
                    ["DICT_6X6_50","DICT_6X6_50"],["DICT_7X7_100","DICT_7X7_100"],["DICT_7X7_1000","DICT_7X7_1000"],
                    ["DICT_7X7_250","DICT_7X7_250"],["DICT_7X7_50","DICT_7X7_50"],["DICT_APRILTAG_16H5","DICT_APRILTAG_16H5"],
                    ["DICT_APRILTAG_16h5","DICT_APRILTAG_16h5"],["DICT_APRILTAG_25H9","DICT_APRILTAG_25H9"],
                    ["DICT_APRILTAG_25h9","DICT_APRILTAG_25h9"],["DICT_APRILTAG_36H10","DICT_APRILTAG_36H10"],
                    ["DICT_APRILTAG_36H11","DICT_APRILTAG_36H11"],["DICT_APRILTAG_36h10","DICT_APRILTAG_36h10"],
                    ["DICT_APRILTAG_36h11","DICT_APRILTAG_36h11"],["DICT_ARUCO_ORIGINAL","DICT_ARUCO_ORIGINAL"]]
                    },
                    {
                    "type": "input_value",
                    "name": "ARUCO_ID",
                    "check": "Number"
                    },
                    {
                    "type": "input_value",
                    "name": "MARKER_SIZE"
                    },
                    {
                    "type": "input_value",
                    "name": "ROI"
                    },
                    {
                    "type": "input_dummy"
                    }
                ],
                "output": None,
                "colour": 180,
                "tooltip": "Detect aruco markers using camera",
                "helpUrl": ""
                },
        sandbox_function = (sandbox_functions.vision_detect_aruco,"ARUCO_DICT", "ARUCO_ID", "MARKER_SIZE", "ROI")
    )

    add_blockly_block(blocks,
        category = "Vision",
        blockly_json = {
                "type": "vision_aruco_detected_get_pose",
                "message0": "get detected aruco marker pose %1",
                "args0": [
                    {
                    "type": "input_value",
                    "name": "DETECTED_MARKER"
                    }
                ],
                "output": None,
                "colour": 180,
                "tooltip": "Get pose of a previously detected marker",
                "helpUrl": ""
                },
        sandbox_function=(sandbox_functions.vision_aruco_detected_get_pose,"DETECTED_MARKER")
    )

    add_blockly_block(blocks,
        category = "Vision",
        blockly_json = {
                "type": "vision_aruco_detected_get_id",
                "message0": "get detected aruco marker id %1",
                "args0": [
                    {
                    "type": "input_value",
                    "name": "DETECTED_MARKER"
                    }
                ],
                "output": None,
                "colour": 180,
                "tooltip": "Get id of a previously detected marker",
                "helpUrl": ""
                },
        sandbox_function = (sandbox_functions.vision_aruco_detected_get_id, "DETECTED_MARKER")
    )

    add_blockly_block(blocks,
        category = "Vision",
        blockly_json = {
                "type": "vision_template_match",
                "message0": "vision match template name %1 object z (m) %2 ROI %3 %4",
                "args0": [
                    {
                    "type": "input_value",
                    "name": "TEMPLATE"
                    },
                    {
                    "type": "input_value",
                    "name": "OBJECT_Z"
                    },
                    {
                    "type": "input_value",
                    "name": "ROI"
                    },
                    {
                    "type": "input_dummy"
                    }
                ],
                "output": None,
                "colour": 180,
                "tooltip": "Object template matching using cross-correlation",
                "helpUrl": ""
                },
        sandbox_function=(sandbox_functions.vision_template_match, "TEMPLATE","OBJECT_Z","ROI")
    )

    add_blockly_block(blocks,
        category = "Vision",
        blockly_json = {
                "type": "vision_matched_template_get_pose",
                "message0": "get template match pose %1",
                "args0": [
                    {
                    "type": "input_value",
                    "name": "TEMPLATE_MATCH"
                    }
                ],
                "output": None,
                "colour": 180,
                "tooltip": "Get pose of a previously matched template",
                "helpUrl": ""
                },
        sandbox_function = (sandbox_functions.vision_matched_template_get_pose,"TEMPLATE_MATCH")
    )

    return blocks

def _get_categories() -> Dict[str,PyriBlocklyCategory]:
    categories = {}
    add_blockly_category(categories, "Vision", 180)
    return categories

class PyriVisionBlocklyPluginFactory(PyriBlocklyPluginFactory):
    def get_plugin_name(self):
        return "pyri-vision"

    def get_category_names(self) -> List[str]:
        return ["Vision"]

    def get_categories(self) -> List[PyriBlocklyCategory]:
        return _get_categories()

    def get_block_names(self) -> List[str]:
        return list(_get_blocks().keys())

    def get_block(self,name) -> PyriBlocklyBlock:
        return _get_blocks()[name]

    def get_all_blocks(self) -> Dict[str,PyriBlocklyBlock]:
        return _get_blocks()

def get_blockly_factory():
    return PyriVisionBlocklyPluginFactory()