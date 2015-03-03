#!/usr/bin/env python

from setuptools import setup, find_packages

setup(name='libreader',
      description='A library for communicating with the Reader module',
      version='0.1',
      url='https://github.com/fmfi-svt-gate/libreader.py',
      license='MIT',
      packages=find_packages(),
      install_requires=['pyserial'])
