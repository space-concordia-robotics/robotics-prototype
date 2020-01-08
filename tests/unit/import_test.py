#!/usr/bin/env python3
import pytest

def test_custom_import():
    '''
    ensure that build suceeded after running `python setup.py develop`
    allowing for our custom modules to be imported anywhere in our repository
    '''
    from robot.basestation.app import app
    assert not (app is None)

def test_non_existant_import():
    with pytest.raises(ImportError):
        from robot.basestation.nonexistantmodule import nonexistantmodule
        assert (app is None)
