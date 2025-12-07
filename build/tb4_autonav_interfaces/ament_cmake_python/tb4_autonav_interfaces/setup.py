from setuptools import find_packages
from setuptools import setup

setup(
    name='tb4_autonav_interfaces',
    version='0.0.1',
    packages=find_packages(
        include=('tb4_autonav_interfaces', 'tb4_autonav_interfaces.*')),
)
