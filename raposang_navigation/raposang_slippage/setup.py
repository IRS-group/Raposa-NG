#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['raposang_slippage', 'raposang_slippage_ros'],
 package_dir={'raposang_slippage': 'common/src/raposang_slippage', 'raposang_slippage_ros': 'ros/src/raposang_slippage_ros'}
)

setup(**d)
