from setuptools import find_packages
from setuptools import setup

setup(
    name='traj_utils',
    version='0.0.0',
    packages=find_packages(
        include=('traj_utils', 'traj_utils.*')),
)
