$(document).ready(function () {

  const KEY_A = 65;
  const KEY_D = 68;
  const KEY_W = 87;
  const KEY_S = 83;

  let keys = {};

  $(document).keydown(function (e) {
    keys[e.which] = true;

    update_movement();
    toggle_drive_buttons(e.which, true);
  });

  $(document).keyup(function (e) {
    delete keys[e.which];

    update_movement();
    toggle_drive_buttons(e.which, false);
  });

  function driveButtonUp(button)
  {
    $(button).css('background-color', GREEN)
  }

  function toggle_drive_buttons(key, turnOn)
  {
    let uiFunc = turnOn ? dim : driveButtonUp;

    switch (key)
    {
      case KEY_W:
        uiFunc("#rover-up > button")
        break;
      case KEY_A:
        uiFunc("#rover-left > button")
        break;
      case KEY_S:
        uiFunc("#rover-down > button")
        break;
      case KEY_D:
        uiFunc("#rover-right > button")
        break;
    }
  }

  setInterval(update_movement, 200);

  twist_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_vel',
    messageType: 'geometry_msgs/Twist'
  })

  /*
 * Sends a Twist command to move the rover.
 * linear : the linear speed (m/s) that the rover should move at (negative means moving backwards)
 * angular : the angular speed (rad/s) that the rover should turn left (negative means turning right)
 */
  function move_rover(linear, angular)
  {
    let twistMessage = new ROSLIB.Message({
      linear : {
        x : linear,
        y : 0,
        z : 0
      },
      angular : {
        x : 0,
        y : 0,
        z : angular
      }
    });

    twist_topic.publish(twistMessage);
  }

  /*
   * Looks periodically at the key presses, button presses and throttle values to determine
   * with what values a Twist command should be sent.
   */
  function update_movement()
  {
    //todo: remove todos before PR.

    //todo: put all this into a new javascript file called rover_controls.js

    let desiredLinearSpeed = 0.5; // todo speed should based on throttle
    let desiredAngularSpeed = 1; // todo angular speed should be based on throttle

    let currentLinearSpeed = 0;
    let currentAngularSpeed = 0;

    if(keys[KEY_A])
      currentAngularSpeed += desiredAngularSpeed;

    if(keys[KEY_D])
      currentAngularSpeed -= desiredAngularSpeed;

    if(keys[KEY_W])
      currentLinearSpeed += desiredLinearSpeed;

    if(keys[KEY_S])
      currentLinearSpeed -= desiredLinearSpeed;

    move_rover(currentLinearSpeed, currentAngularSpeed);
  }


})
