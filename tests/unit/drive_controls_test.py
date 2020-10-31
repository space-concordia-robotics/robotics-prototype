from robot.rospackages.src.mcu_control.scripts.RoverNode import twist_to_rover_command
import robot.rospackages.src.mcu_control.scripts.RoverNode as rn

def test_twist_to_rover():
    zero = twist_to_rover_command(0, 0)
    assert(zero == "0:0")

    max_positive_speed = twist_to_rover_command(rn.max_speed, rn.max_angular_speed)
    assert(max_positive_speed == str(rn.max_throttle) + ":" + str(rn.max_steering))

    max_negative_speed = twist_to_rover_command(-rn.max_speed, -rn.max_angular_speed)
    assert(max_negative_speed == str(-rn.max_throttle) + ":" + str(-rn.max_steering))

    half_max_positive =  twist_to_rover_command(rn.max_speed * 0.5, rn.max_angular_speed * 0.5)

    assert(half_max_positive == str(round(rn.max_throttle * 0.5)) + ":" + str(round(rn.max_steering * 0.5)))

    half_max_negative =  twist_to_rover_command(-rn.max_speed * 0.5, -rn.max_angular_speed * 0.5)

    assert(half_max_negative == str(round(rn.max_throttle * -0.5)) + ":" + str(round(rn.max_steering * -0.5)))
