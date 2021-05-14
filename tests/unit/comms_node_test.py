from robot.rospackages.src.mcu_control.scripts.CommsNode import parse_command, get_arg_bytes

def test_parse_command():
    """
    test the parsing of commands
    """
    assert parse_command("arm estop") == ("estop", [])
    assert parse_command("arm arm_speed 2.25") == ("arm_speed", [2.25])
    assert parse_command("arm pid_constants 2 1.11 4.20 6.90") == ("pid_constants", [2, 1.11, 4.20, 6.90])
    assert parse_command("arm string_test 2 test 1.12 9.60") == ("string_test", [2, "test", 1.12, 9.60])

def test_get_arg_bytes():
    """
    test total number of bytes in args calculation
    """
    assert get_arg_bytes(("estop", 0, [])) == 0
    assert get_arg_bytes(("arm_speed", 0, [4])) == 4
    assert get_arg_bytes(("pid_constants", 0, [1, 4, 4, 4])) == 13
