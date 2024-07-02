from setuptools import setup

package_name = 'ur5e_joystick_control'

#!/usr/bin/env python
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['ur5e_joystick_control'],  # Replace with your package name
    package_dir={'': 'src'},  # Directory where your Python package source code is located
    scripts=['scripts/ur5e_inverse_kinematics.py']  # Path to your executable scripts
)

setup(**setup_args)

