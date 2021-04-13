from setuptools import setup, find_packages, find_namespace_packages

setup(
    name='pyri-vision',
    version='0.1.0',
    description='PyRI Teach Pendant Vision Plugin',
    author='John Wason',
    author_email='wason@wasontech.com',
    url='http://pyri.tech',
    package_dir={'': 'src'},
    packages=find_namespace_packages(where='src'),
    include_package_data=True,
    zip_safe=False,
    install_requires=[
        'pyri-common',
        'opencv-python'
    ],
    tests_require=['pytest','pytest-asyncio'],
    extras_require={
        'test': ['pytest','pytest-asyncio']
    },
    entry_points = {
        'pyri.plugins.sandbox_functions': ['pyri-vision-sandbox-functions=pyri.vision.sandbox_functions:get_sandbox_functions_factory'],
        'pyri.plugins.device_type_adapter': ['pyri-vision-type-adapter = pyri.vision.device_type_adapter:get_device_type_adapter_factory'],
        'pyri.plugins.blockly': ['pyri-vision-plugin-blockly=pyri.vision.blockly:get_blockly_factory']
    }
)