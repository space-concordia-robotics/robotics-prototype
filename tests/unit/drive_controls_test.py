from robot.rospackages.src.mcu_control.scripts.DriveControls import twist_to_rover_command, accelerate_value
import robot.rospackages.src.mcu_control.scripts.DriveControls as dc
from geometry_msgs.msg import Twist

def test_twist_to_rover_zero():
    zero = twist_to_rover_command(0, 0)
    assert(zero == "0:0")

def test_twist_to_rover_overshoot():
    assert(twist_to_rover_command(2*dc.max_speed, 2 * dc.max_angular_speed) == twist_to_rover_command(dc.max_speed, dc.max_angular_speed)) 
    assert(twist_to_rover_command(2*-dc.max_speed, 2 * -dc.max_angular_speed) == twist_to_rover_command(-dc.max_speed, -dc.max_angular_speed)) 

def test_acceleration():
    assert(accelerate_value(0.3, 0.5, 0.1, 0) == 0.3)
    assert(accelerate_value(0.3, 0.5, 0.1, 1000) == 0.4)
    assert(accelerate_value(0.2, 0.6, 0.5, 500) == 0.45)
    assert(accelerate_value(0.3, 0.5, 10, 1000) == 0.5)
    assert(accelerate_value(0, -0.2, 0.1, 250) == -0.025)
    assert(accelerate_value(-0.5, -0.5, 0.1, 250) == -0.5)
    assert(accelerate_value(0.3, 0, 0.1, 250) == 0)

