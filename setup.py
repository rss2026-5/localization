import glob
import os
from setuptools import setup, find_packages, Extension

package_name = 'localization'

# Build the Cython scan_simulator_2d extension only when the .pyx source and
# the simulator library are available (i.e. in the sim environment with SIM_WS).
# On the real car the module is already pre-built in the Docker image.
ext_modules = []
pyx_path = os.path.join(package_name, "scan_simulator_2d.pyx")

if os.path.isfile(pyx_path):
    try:
        from Cython.Build import cythonize

        if "SIM_WS" in os.environ:
            prefix = os.path.join(os.environ["SIM_WS"], "install", "racecar_simulator")
        else:
            prefix = os.path.join(os.path.expanduser("~"), "racecar_ws", "install", "racecar_simulator")

        extensions = Extension(
            "scan_simulator_2d",
            [pyx_path],
            language="c++",
            libraries=["racecar_simulator"],
            include_dirs=[os.path.join(prefix, "include")],
            library_dirs=[os.path.join(prefix, "lib")],
            extra_compile_args=['-Wno-cpp', '-g', '-Wno-maybe-uninitialized'],
        )
        ext_modules = cythonize(extensions, force=True, quiet=True)
    except Exception:
        pass

setup(
    ext_modules=ext_modules,
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'localization/params.yaml', 'localization/test/test_params.yaml', 'localization/real_params.yaml', 'localization/rviz_config.rviz']),
        ('share/localization/launch',
         glob.glob(os.path.join('launch', '*launch.*')) + glob.glob(os.path.join('launch/unit_tests', '*launch.*'))),
        ('share/localization/test_map', [f for f in glob.glob(os.path.join('test_map', '*')) if os.path.isfile(f)]),
        ('share/localization/maps', [f for f in glob.glob(os.path.join('maps', '*')) if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alanyu',
    maintainer_email='alanyu@csail.mit.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'particle_filter = localization.particle_filter:main',
            'sensor_model_test = localization.test.sensor_model_test:main',
            'motion_model_test = localization.test.motion_model_test:main',
        ],
    },
)
