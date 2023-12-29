from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# required packages
req_packages = [
    'pyrealsense2>=2.5.0',
]

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['cgras_robot'],
    package_dir={'': 'src'},
    python_requires='>=3.9',
    install_requires=req_packages
)

setup(**setup_args)
