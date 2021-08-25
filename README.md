<p align="center">
<img src="./doc/figures/pyri_logo_web.svg" height="200"/>
</p>

# PyRI Open Source Teach Pendant Vision Package

This package is part of the PyRI project. See https://github.com/pyri-project/pyri-core#documentation for documentation. This package is included in the `pyri-robotics-superpack` Conda package.

The `pyri-vision` package contains plugins and services for computer vision operations.

Demo videos:

* https://youtu.be/0Q6a07FSsBc
* https://youtu.be/jF_BGaFI7Qc

## Blocks and Sandbox Functions

This package contains Blockly blocks and PyRI sandbox functions for computer vision operations. See [vision_blocks_functions.md](doc/vision_blocks_functions.md) for more information.

## Services

`pyri-vision` provides five services:

### pyri-vision-aruco-detection-service

The `pyri-vision-aruco-detection-service` detects OpenCV ArUco markers in camera frames and images.

This service is started automatically by `pyri-core`, and does not normally need to be started manually.

Standalone service command line example:

```
pyri-vision-aruco-detection-service
```

The `pyri-variable-storage` and `pyri-device-manager` services must be running before use.

Command line options:

| Option | Type | Required | Description |
| ---    | ---  | ---      | ---         |
| `--device-manager-url=` | Robot Raconteur URL | No | Robot Raconteur URL of device manager service |
| `--device-manager-identifier=` | Identifier | No | Robot Raconteur device identifier in string format for device manager service |
| `--device-info-file=` | File | No | Robot Raconteur `DeviceInfo` YAML file. Defaults to contents of `pyri_vision_aruco_detection_service_default_info.yml` |

This service may use any standard `--robotraconteur-*` service node options.

This service uses the `DeviceManagerClient`, which needs to connect to the device manager service to find other devices. This can be done using discovery based on a Robot Raconteur device identifier, or using a specified Robot Raconteur URL. If neither is specified, the `DeviceManagerClient` will search for the identifier named `pyri_device_manager` on the local machine.

### pyri-vision-camera-calibration-service

The `pyri-vision-camera-calibration-service` is used to calibrate intrinsic and extrinsic camera parameters.

This service is started automatically by `pyri-core`, and does not normally need to be started manually.

Standalone service command line example:

```
pyri-vision-camera-calibration-service
```

The `pyri-variable-storage` and `pyri-device-manager` services must be running before use.

Command line options:

| Option | Type | Required | Description |
| ---    | ---  | ---      | ---         |
| `--device-manager-url=` | Robot Raconteur URL | No | Robot Raconteur URL of device manager service |
| `--device-manager-identifier=` | Identifier | No | Robot Raconteur device identifier in string format for device manager service |
| `--device-info-file=` | File | No | Robot Raconteur `DeviceInfo` YAML file. Defaults to contents of `pyri_vision_camera_calibration_service_default_info.yml` |

This service may use any standard `--robotraconteur-*` service node options.

This service uses the `DeviceManagerClient`, which needs to connect to the device manager service to find other devices. This can be done using discovery based on a Robot Raconteur device identifier, or using a specified Robot Raconteur URL. If neither is specified, the `DeviceManagerClient` will search for the identifier named `pyri_device_manager` on the local machine.

### pyri-vision-camera-viewer-service

The `pyri-vision-camera-viewer-service` is used to support the live camera viewing and image frame capture in the WebUI.

This service is started automatically by `pyri-core`, and does not normally need to be started manually.

Standalone service command line example:

```
pyri-vision-camera-viewer-service
```

The `pyri-variable-storage` and `pyri-device-manager` services must be running before use.

Command line options:

| Option | Type | Required | Description |
| ---    | ---  | ---      | ---         |
| `--device-manager-url=` | Robot Raconteur URL | No | Robot Raconteur URL of device manager service |
| `--device-manager-identifier=` | Identifier | No | Robot Raconteur device identifier in string format for device manager service |
| `--device-info-file=` | File | No | Robot Raconteur `DeviceInfo` YAML file. Defaults to contents of `pyri_vision_camera_viewer_service_default_info.yml` |

This service may use any standard `--robotraconteur-*` service node options.

This service uses the `DeviceManagerClient`, which needs to connect to the device manager service to find other devices. This can be done using discovery based on a Robot Raconteur device identifier, or using a specified Robot Raconteur URL. If neither is specified, the `DeviceManagerClient` will search for the identifier named `pyri_device_manager` on the local machine.

### pyri-vision-robot-calibration-service

The `pyri-vision-robot-calibration-service` is used to calibrate robots in the workspace using computer vision.

This service is started automatically by `pyri-core`, and does not normally need to be started manually.

Standalone service command line example:

```
pyri-vision-robot-calibration-service
```

The `pyri-variable-storage` and `pyri-device-manager` services must be running before use.

Command line options:

| Option | Type | Required | Description |
| ---    | ---  | ---      | ---         |
| `--device-manager-url=` | Robot Raconteur URL | No | Robot Raconteur URL of device manager service |
| `--device-manager-identifier=` | Identifier | No | Robot Raconteur device identifier in string format for device manager service |
| `--device-info-file=` | File | No | Robot Raconteur `DeviceInfo` YAML file. Defaults to contents of `pyri_vision_robot_calibration_service_default_info.yml` |

This service may use any standard `--robotraconteur-*` service node options.

This service uses the `DeviceManagerClient`, which needs to connect to the device manager service to find other devices. This can be done using discovery based on a Robot Raconteur device identifier, or using a specified Robot Raconteur URL. If neither is specified, the `DeviceManagerClient` will search for the identifier named `pyri_device_manager` on the local machine.

### pyri-vision-template-matching-service

The `pyri-vision-template-matching-service` is used to detect objects using planar template matching in images and camera frames.

This service is started automatically by `pyri-core`, and does not normally need to be started manually.

Standalone service command line example:

```
pyri-vision-template-matching-service
```

The `pyri-variable-storage` and `pyri-device-manager` services must be running before use.

Command line options:

| Option | Type | Required | Description |
| ---    | ---  | ---      | ---         |
| `--device-manager-url=` | Robot Raconteur URL | No | Robot Raconteur URL of device manager service |
| `--device-manager-identifier=` | Identifier | No | Robot Raconteur device identifier in string format for device manager service |
| `--device-info-file=` | File | No | Robot Raconteur `DeviceInfo` YAML file. Defaults to contents of `pyri_vision_template_matching_service_default_info.yml` |

This service may use any standard `--robotraconteur-*` service node options.

This service uses the `DeviceManagerClient`, which needs to connect to the device manager service to find other devices. This can be done using discovery based on a Robot Raconteur device identifier, or using a specified Robot Raconteur URL. If neither is specified, the `DeviceManagerClient` will search for the identifier named `pyri_device_manager` on the local machine.

## Acknowledgment

This work was supported in part by Subaward No. ARM-TEC-19-01-F-24 from the Advanced Robotics for Manufacturing ("ARM") Institute under Agreement Number W911NF-17-3-0004 sponsored by the Office of the Secretary of Defense. ARM Project Management was provided by Christopher Adams. The views and conclusions contained in this document are those of the authors and should not be interpreted as representing the official policies, either expressed or implied, of either ARM or the Office of the Secretary of Defense of the U.S. Government. The U.S. Government is authorized to reproduce and distribute reprints for Government purposes, notwithstanding any copyright notation herein.

This work was supported in part by the New York State Empire State Development Division of Science, Technology and Innovation (NYSTAR) under contract C160142. 

![](doc/figures/arm_logo.jpg) ![](doc/figures/nys_logo.jpg)

PyRI is developed by Rensselaer Polytechnic Institute, Wason Technology, LLC, and contributors.