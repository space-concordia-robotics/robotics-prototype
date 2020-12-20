from robot.rospackages.src.mcu_control.scripts.DriveControls import twist_to_rover_command, accelerate_value
import robot.rospackages.src.mcu_control.scripts.DriveControls as dc
from geometry_msgs.msg import Twist

def test_twist_to_rover():
    zero = twist_to_rover_command(0, 0)
    assert(zero == "0:0")

    max_positive_speed = twist_to_rover_command(dc.max_speed, dc.max_angular_speed)
    assert(max_positive_speed == str(dc.max_throttle) + ":" + str(dc.max_steering))

    max_negative_speed = twist_to_rover_command(-dc.max_speed, -dc.max_angular_speed)
    assert(max_negative_speed == str(-dc.max_throttle) + ":" + str(-dc.max_steering))

    half_max_positive =  twist_to_rover_command(dc.max_speed * 0.5, dc.max_angular_speed * 0.5)

    assert(half_max_positive == str(round(dc.max_throttle * 0.5)) + ":" + str(round(dc.max_steering * 0.5)))

    half_max_negative =  twist_to_rover_command(-dc.max_speed * 0.5, -dc.max_angular_speed * 0.5)

    assert(half_max_negative == str(round(dc.max_throttle * -0.5)) + ":" + str(round(dc.max_steering * -0.5)))

    assert(twist_to_rover_command(2*dc.max_speed, 2 * dc.max_angular_speed) ==twist_to_rover_command(dc.max_speed, dc.max_angular_speed)) 
    assert(twist_to_rover_command(2*-dc.max_speed, 2 * -dc.max_angular_speed) ==twist_to_rover_command(-dc.max_speed, -dc.max_angular_speed)) 

def test_acceleration():
    assert(accelerate_value(0.3, 0.5, 0.1, 0) == 0.3)
    assert(accelerate_value(0.3, 0.5, 0.1, 1000) == 0.4)
    assert(accelerate_value(0.2, 0.6, 0.5, 500) == 0.45)
    assert(accelerate_value(0.3, 0.5, 10, 1000) == 0.5)
    assert(accelerate_value(0, -0.2, 0.1, 250) == -0.025)
    assert(accelerate_value(-0.5, -0.5, 0.1, 250) == -0.5)
    assert(accelerate_value(0.3, 0, 0.1, 250) == 0)

