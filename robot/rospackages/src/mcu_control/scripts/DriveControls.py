import time
from numpy import interp
last_angular_speed = None
last_linear_speed = None
last_speed_change_ms = None

linear_acceleration_rate = 0.5
angular_acceleration_rate = 0.75

max_speed = 0.5 # Maximum rover speed in (m/s)
max_angular_speed = 1 # Maximum angular speed in (rad/s)

max_throttle = 255
max_steering = 255

expire_rate = 300 # A command lasts for 300 ms unless overwritten
initial_ramp_factor = 3 

def accelerate_twist(twist):
    global last_speed_change_ms
    global last_speed_change_ms
    global last_linear_speed
    global last_angular_speed
    
    is_expired = False
    
    twist_linear = twist.linear.x
    twist_angular = twist.angular.z

    if last_speed_change_ms is not None:
        is_expired = (time.time()*1000 - last_speed_change_ms) > expire_rate

    if last_speed_change_ms is None or is_expired:
        last_linear_speed = 0
        last_angular_speed = 0
        last_speed_change_ms = time.time()*1000 - expire_rate / initial_ramp_factor

    delta = (time.time()*1000) - last_speed_change_ms
    
    linear = accelerate_value(last_linear_speed, twist_linear, linear_acceleration_rate, delta)
    angular = accelerate_value(last_angular_speed, twist_angular, angular_acceleration_rate, delta)
    
    last_linear_speed = linear
    last_angular_speed = angular
    last_speed_change_ms = time.time() * 1000

    return linear, angular

def accelerate_value(current, desired, rate, dt):
    """
    Accelerates the current speed to a desired speed at a certain rate while
    considering a certain time difference. Ex : Current Speed 0.3 m/s, desired speed 0.5 m/s,
    if the rate is 0.1 m/s^2 and dt is 100 milliseconds then the new speed should be 0.31 m/s.
    """
    if(desired == current):
        return desired

    if(desired == 0):
        return 0

    if(desired < current):
        rate = -rate

    new_value = current + rate * dt /1000
    if(abs(new_value) > abs(desired)):
        new_value = desired
    return new_value

def twist_to_rover_command(linear, angular):
    """ 
    Converts Twist command to serial 
    """

    if linear > max_speed:
        linear = max_speed
    elif linear < -max_speed:
        linear = -max_speed

    if angular > max_angular_speed:
        angular = max_angular_speed
    elif angular < -max_angular_speed:
        angular = -max_angular_speed

    throttle = interp(linear,[-max_speed,max_speed],[-255,255])
    steering = interp(angular,[-max_angular_speed,max_angular_speed],[-255,255])
    
    args = []

    if throttle < 0 :
        args.append(0)
    else :
        args.append(1)

    args.append(abs(throttle))

    if steering < 0 :
        args.append(0)
    else :
        args.append(1)

    args.append(abs(steering))

    print(str(args))
    return args
