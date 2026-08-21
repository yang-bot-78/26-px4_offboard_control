from setuptools import find_packages
from setuptools import setup

setup(
    name='quadrotor_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('quadrotor_msgs', 'quadrotor_msgs.*')),
)
