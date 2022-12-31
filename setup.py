from setuptools import setup, find_packages, find_namespace_packages

setup(
    package_dir={'': 'src'},
    packages=find_namespace_packages(where='src'),
    include_package_data=True,
    package_data = {
        'pyri.vision.aruco_detection_service': ['*.robdef','*.yml'],
        'pyri.vision.camera_calibration_service': ['*.robdef','*.yml'],
        'pyri.vision.camera_viewer_service': ['*.robdef','*.yml'],
        'pyri.vision.robot_calibration_service': ['*.robdef','*.yml'],
        'pyri.vision.template_matching': ['*.robdef','*.yml']
    },
    zip_safe=False
)