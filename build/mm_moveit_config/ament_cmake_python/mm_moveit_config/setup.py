from setuptools import find_packages
from setuptools import setup

setup(
    name='mm_moveit_config',
    version='2.5.2',
    packages=find_packages(
        include=('mm_moveit_config', 'mm_moveit_config.*')),
)
