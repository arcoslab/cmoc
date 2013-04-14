#!/usr/bin/env python

from distutils.core import setup


setup(name='cmoc',
      version='0.1',
      description='Compact Models for Object Control',
      author='Federico Ruiz Ugalde',
      author_email='memeruiz@gmail.com',
      url='http://www.arcoslab.org/',
      package_dir={'cmoc': ''},
      packages=['cmoc', 'cmoc.robot', 'cmoc.objects', 'cmoc.objects.sliding'],
      scripts=['robot/joint_sim', 'robot/torque_sim', 'robot/sahand_yarp_sim', 'objects/sliding/planar_sliding_simple', 'objects/sliding/tools/xfinger_feeder', 'objects/sliding/planar_sliding', 'objects/sliding/slider_control', 'objects/sliding/slider_control_simple']
     )

