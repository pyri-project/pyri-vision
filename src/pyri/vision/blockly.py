from pyri.plugins.blockly import PyriBlocklyPluginFactory, PyriBlocklyBlock, PyriBlocklyCategory
from typing import List, Dict, NamedTuple, TYPE_CHECKING

def _get_blocks() -> Dict[str,PyriBlocklyBlock]:
    blocks = {}

    blocks["vision_detect_aruco"] = PyriBlocklyBlock(
        name = "vision_detect_aruco",
        category = "Vision",
        doc = "Detect aruco markers using camera",
        json = """
               {
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
                "output": null,
                "colour": 180,
                "tooltip": "Detect aruco markers using camera",
                "helpUrl": ""
                }
               """,
        python_generator = """
                            Blockly.Python['vision_detect_aruco'] = function(block) {
                            var dropdown_aruco_dict = block.getFieldValue('ARUCO_DICT');
                            var value_aruco_id = Blockly.Python.valueToCode(block, 'ARUCO_ID', Blockly.Python.ORDER_ATOMIC);
                            var value_marker_size = Blockly.Python.valueToCode(block, 'MARKER_SIZE', Blockly.Python.ORDER_ATOMIC);
                            var value_roi = Blockly.Python.valueToCode(block, 'ROI', Blockly.Python.ORDER_ATOMIC);
                            // TODO: Assemble Python into code variable.
                            var code = 'vision_detect_aruco(\"' + dropdown_aruco_dict + '\",' + value_aruco_id + ',' + value_marker_size + ',' + value_roi + ')';
                            // TODO: Change ORDER_NONE to the correct strength.
                            return [code, Blockly.Python.ORDER_NONE];
                            };
                           """
    )

    blocks["vision_aruco_detected_get_pose"] = PyriBlocklyBlock(
        name = "vision_aruco_detected_get_pose",
        category = "Vision",
        doc = "Get the pose of a previously detected marker",
        json = """
                {
                "type": "vision_aruco_detected_get_pose",
                "message0": "get detected aruco marker pose %1",
                "args0": [
                    {
                    "type": "input_value",
                    "name": "DETECTED_MARKER"
                    }
                ],
                "output": null,
                "colour": 180,
                "tooltip": "Get pose of a previously detected marker",
                "helpUrl": ""
                }
               """,
        python_generator = """
                            Blockly.Python['vision_aruco_detected_get_pose'] = function(block) {
                            var value_detected_marker = Blockly.Python.valueToCode(block, 'DETECTED_MARKER', Blockly.Python.ORDER_ATOMIC);
                            // TODO: Assemble Python into code variable.
                            var code = 'vision_aruco_detected_get_pose(' + value_detected_marker + ')';
                            // TODO: Change ORDER_NONE to the correct strength.
                            return [code, Blockly.Python.ORDER_NONE];
                            };
                           """
    )

    blocks["vision_aruco_detected_get_id"] = PyriBlocklyBlock(
        name = "vision_aruco_detected_get_id",
        category = "Vision",
        doc = "Get the id of a previously detected marker",
        json = """
                {
                "type": "vision_aruco_detected_get_id",
                "message0": "get detected aruco marker id %1",
                "args0": [
                    {
                    "type": "input_value",
                    "name": "DETECTED_MARKER"
                    }
                ],
                "output": null,
                "colour": 180,
                "tooltip": "Get id of a previously detected marker",
                "helpUrl": ""
                }
               """,
        python_generator = """
                            Blockly.Python['vision_aruco_detected_get_id'] = function(block) {
                            var value_detected_marker = Blockly.Python.valueToCode(block, 'DETECTED_MARKER', Blockly.Python.ORDER_ATOMIC);
                            // TODO: Assemble Python into code variable.
                            var code = 'vision_aruco_detected_get_id(' + value_detected_marker + ')';
                            // TODO: Change ORDER_NONE to the correct strength.
                            return [code, Blockly.Python.ORDER_NONE];
                            };
                           """
    )

    blocks["vision_template_match"] = PyriBlocklyBlock(
        name = "vision_template_match",
        category = "Vision",
        doc = "Object template matching using cross-correlation",
        json = """
                {
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
                "output": null,
                "colour": 180,
                "tooltip": "Run vision template matching",
                "helpUrl": ""
                }
               """,
        python_generator = """
                            Blockly.Python['vision_template_match'] = function(block) {
                            var value_template = Blockly.Python.valueToCode(block, 'TEMPLATE', Blockly.Python.ORDER_ATOMIC);
                            var value_object_z = Blockly.Python.valueToCode(block, 'OBJECT_Z', Blockly.Python.ORDER_ATOMIC);
                            var value_roi = Blockly.Python.valueToCode(block, 'ROI', Blockly.Python.ORDER_ATOMIC);
                            // TODO: Assemble Python into code variable.
                            var code = 'vision_template_match(' + value_template + ',' + value_object_z + ',' + value_roi +')';
                            // TODO: Change ORDER_NONE to the correct strength.
                            return [code, Blockly.Python.ORDER_NONE];
                            };
                           """
    )

    blocks["vision_matched_template_get_pose"] = PyriBlocklyBlock(
        name = "vision_matched_template_get_pose",
        category = "Vision",
        doc = "Get pose of a previous template match",
        json = """
                {
                "type": "vision_matched_template_get_pose",
                "message0": "get template match pose %1",
                "args0": [
                    {
                    "type": "input_value",
                    "name": "TEMPLATE_MATCH"
                    }
                ],
                "output": null,
                "colour": 180,
                "tooltip": "Get pose of a previously matched template",
                "helpUrl": ""
                }
               """,
        python_generator = """
                            Blockly.Python['vision_matched_template_get_pose'] = function(block) {
                            var value_template_match = Blockly.Python.valueToCode(block, 'TEMPLATE_MATCH', Blockly.Python.ORDER_ATOMIC);
                            // TODO: Assemble Python into code variable.
                            var code = 'vision_matched_template_get_pose(' + value_template_match + ')';
                            // TODO: Change ORDER_NONE to the correct strength.
                            return [code, Blockly.Python.ORDER_NONE];
                            };
                           """
    )

    return blocks

def _get_categories() -> Dict[str,PyriBlocklyCategory]:
    categories = {}
    categories["Vision"] = PyriBlocklyCategory(
        name = "Vision",
        json = '{"kind": "category", "name": "Vision", "colour": 230 }'
    )

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