last_angular_speed = None
last_linear_speed = None
last_speed_change_ms = None
acceleration_rate = 0.1 # m/s^2
max_speed = 0.5 # Maximum rover speed in (m/s) NEEDS TO BE TWEAKED
max_angular_speed = 1 # Maximum angular speed in (rad/s) NEEDS TO BE TWEAKED

max_throttle = 49
max_steering = 49

expire_rate = 300 # A command lasts for 300 ms unless overwritten
initial_ramp_factor = 3 

def accelerate_twist(twist):
    global last_speed_change_ms
    global last_linear_speed
    global last_angular_speed

    twist_linear = twist.linear.x
    twist_angular = twist.angular.z

    is_expired = (time.time_ns()/1000 - last_speed_change_ms) > expire_rate

    if last_speed_change_ns == None or is_expired:
        last_linear_speed = 0
        last_angular_speed = 0
        last_speed_change_ms = time.time_ns()/1000 - expire_rate / initial_ramp_factor

    delta = (time.time_ns()/1000) - last_speed_change_ms
    
    linear = accelerate_value(last_linear_speed, twist_linear, rate_linear, delta)
    angular = accelerate_value(last_angular_speed, twist_angular, rate_angular, delta)

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

    linear_speed_bounds = [-max_speed, max_speed]
    angular_speed_bounds = [-max_angular_speed, max_angular_speed]

    throttle_bounds = [-max_throttle, max_throttle]
    steering_bounds = [-max_steering, max_steering]

    throttle = linear / max_speed # throttle should now be in [-1, 1]
    steering = angular / max_angular_speed # steering should now [-1,1]

    linear_motor_val = throttle * max_throttle
    angular_motor_val = steering * max_steering

    return str(round(linear_motor_val)) + ':' + str(round(angular_motor_val))

