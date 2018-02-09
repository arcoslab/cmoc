# -*- coding: utf-8 -*-
"""
Test suite for module python_boilerplate.

See http://pythontesting.net/framework/pytest/pytest-introduction/#fixtures
"""

from __future__ import unicode_literals, absolute_import
from __future__ import print_function, division

import pytest  # noqa

from cmoc import __version__


def setup_module(module):
    print('setup_module({})'.format(module.__name__))


def teardown_module(module):
    print('teardown_module({})'.format(module.__name__))


def test_semantic_version():
    """
    Check that version follows the Semantic Versioning 2.0.0 specification.

        http://semver.org/
    """
    mayor, minor, rev = map(int, __version__.split('.'))

    assert mayor >= 0
    assert minor >= 0
    assert rev >= 0
