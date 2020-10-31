from robot.rospackages.src.mcu_control.scripts.RoverNode import twist_to_rover_command
import robot.rospackages.src.mcu_control.scripts.RoverNode as RoverNode

def test_twist_to_rover():
    zero = twist_to_rover_command(0, 0)
    assert(zero == "0:0")

    max_positive_speed = twist_to_rover_command(RoverNode.max_speed, RoverNode.max_angular_speed)
    assert(max_positive_speed == str(RoverNode.max_throttle) + ":" + str(RoverNode.max_steering))

    max_negative_speed = twist_to_rover_command(-RoverNode.max_speed, -RoverNode.max_angular_speed)
    assert(max_negative_speed == str(-RoverNode.max_throttle) + ":" + str(-RoverNode.max_steering))

    half_max_positive =  twist_to_rover_command(round(RoverNode.max_speed * 0.5), round(RoverNode.max_angular_speed * 0.5))

    assert(half_max_positive == str(RoverNode.max_throttle * 0.5) + ":" + str(RoverNode.max_steering * 0.5))

    half_max_negative =  twist_to_rover_command(-RoverNode.max_speed * 0.5, -RoverNode.max_angular_speed * 0.5)

    assert(half_max_negative == str(RoverNode.max_throttle * -0.5) + ":" + str(RoverNode.max_steering * -0.5))
