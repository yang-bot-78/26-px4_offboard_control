from setuptools import find_packages
from setuptools import setup

setup(
    name='fastlio_global_slam',
    version='0.1.0',
    packages=find_packages(
        include=('fastlio_global_slam', 'fastlio_global_slam.*')),
)
