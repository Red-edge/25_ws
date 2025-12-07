from setuptools import find_packages
from setuptools import setup

setup(
    name='demo_interfaces_test',
    version='0.0.1',
    packages=find_packages(
        include=('demo_interfaces_test', 'demo_interfaces_test.*')),
)
