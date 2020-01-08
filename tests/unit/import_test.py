#!/usr/bin/env python3

def test_custom_import():
    '''
    ensure that build suceeded after running `python setup.py develop`
    allowing for our custom modules to be imported anywhere in our repository
    '''
    from robot.basestation.app import app
    assert not (app is None)
