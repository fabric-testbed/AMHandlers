# coding: utf-8

from setuptools import setup, find_packages
from fabric_am import __VERSION__

NAME = "fabric-am-handlers"
VERSION = __VERSION__
# To install the library, run the following
#
# python setup.py install
#
# prerequisite: setuptools
# http://pypi.python.org/pypi/setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

with open("requirements.txt", "r") as fh:
    requirements = fh.read()

setup(
    name=NAME,
    version=VERSION,
    description="Fabric Aggregate Manager Handlers",
    author="Komal Thareja, Mauricio Tavares",
    author_email="kthare10@renci.org, mtavares@renci.org",
    url="https://github.com/fabric-testbed/AMHandlers",
    keywords=["Fabric Control Framework", "Aggregate Manager Handlers"],
    install_requires=requirements,
    setup_requires=requirements,
    packages=find_packages(),
    include_package_data=True,
    long_description=long_description,
    long_description_content_type="text/markdown",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.7'
)
