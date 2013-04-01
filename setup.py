#!/usr/bin/env python

from distutils.core import setup


setup(name='cmoc',
      version='1.0',
      description='Compact Models for Object Control',
      author='Federico Ruiz Ugalde',
      author_email='memeruiz@gmail.com',
      url='http://www.arcoslab.org/',
      packages=['robot']
      scripts=['robot/joint_sim']
     )
