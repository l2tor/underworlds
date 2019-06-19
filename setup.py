 # -*- coding: utf-8 -*-
import os
from distutils.core import setup

setup(name='underworlds',
      version='0.1',
      license='ISC',
      description='A framework for geometric and temporal representation of robotic worlds',
      author='Séverin Lemaignan',
      author_email='severin.lemaignan@plymouth.ac.uk',
      url='https://github.com/severin-lemaignan/underworlds',
      package_dir = {'': 'src'},
      scripts=['bin/' + f for f in os.listdir('bin')],
      packages=['underworlds', 'underworlds.helpers', 'underworlds.tools'],
      data_files=[('share/doc/underworlds', ['LICENSE', 'README.md'])],
      requires=["pyassimp","grpc"]
      )
