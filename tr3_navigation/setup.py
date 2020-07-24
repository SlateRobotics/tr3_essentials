#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tr3_navigation'],
    package_dir={'': 'src'}
)


setup(**d)
