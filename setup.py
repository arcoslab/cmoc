#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""The setup script."""

from setuptools import setup, find_packages


def read(filename):
    """
    Read a file relative to setup.py location.
    """
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, filename)) as fd:
        return fd.read()


def find_version(filename):
    """
    Find package version in file.
    """
    import re
    content = read(filename)
    version_match = re.search(
        r"^__version__ = ['\"]([^'\"]*)['\"]", content, re.M)
    if version_match:
        return version_match.group(1)
    raise RuntimeError('Unable to find version string.')


def find_requirements(filename):
    """
    Find requirements in file.
    """
    import string
    content = read(filename)
    requirements = []
    for line in content.splitlines():
        line = line.strip()
        if line and line[:1] in string.ascii_letters:
            requirements.append(line)
    return requirements


setup(
    name='python_boilerplate',
    version='0.1.0',
    description='Compact Models for Object Control',
    long_description=read('README.rst'),
    author='Federico Ruiz Ugalde',
    author_email='memeruiz@gmail.com',
    url='http://www.arcoslab.org/',
    package_dir={'': 'src'},
    packages=find_packages('src'),
    scripts=[
        'robot/joint_sim', 'robot/torque_sim', 'robot/sahand_yarp_sim',
        'objects/sliding/scripts/planar_sliding_simple',
        'objects/sliding/tools/xfinger_feeder',
        'objects/sliding/scripts/planar_sliding',
        'objects/sliding/scripts/slider_control',
        'objects/sliding/scripts/slider_control_simple'
    ],
    include_package_data=True,
    install_requires=find_requirements('requirements.txt'),
    license="GNU General Public License v3",
    zip_safe=False,
    keywords='python_boilerplate',
    classifiers=[
        'Development Status :: 2 - Pre-Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',
        'Natural Language :: English',
        "Programming Language :: Python :: 2",
        'Programming Language :: Python :: 2.6',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.3',
        'Programming Language :: Python :: 3.4',
        'Programming Language :: Python :: 3.5',
    ],
    # Testing
    test_suite='tests',
    tests_require=find_requirements('requirements.dev.txt'),
    setup_requires=find_requirements('requirements.dev.txt'),
)
