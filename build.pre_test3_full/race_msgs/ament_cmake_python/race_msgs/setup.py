from setuptools import find_packages
from setuptools import setup

setup(
    name='race_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('race_msgs', 'race_msgs.*')),
)
