from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# this is needed for the catkin_build for properly running ros_handler
d = generate_distutils_setup(
    packages=['dao'],
    scripts=['src/ros_handler.py'],
    package_dir={'': 'src'}
)

setup(**d)
