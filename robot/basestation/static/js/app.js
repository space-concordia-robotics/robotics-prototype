/* eslint-disable no-unused-vars */
jQuery(document).ready((s) => {
  const Site = {
    init() {
      this.bindEventHandlers();
    },
    bindEventHandlers() {
      this.eventHandlers.forEach(eventHandler => this.bindEvent(eventHandler));
    },
    bindEvent(e) {
      let intervalId;
      const binder = () => {
        $.getJSON(e.route, e.handler);
        intervalId = setInterval(() => { $.getJSON(e.route, e.handler); }, 100);
        return false;
      };
      e.$el.bind(e.event, binder);
      e.$el.bind('mouseup', () => {
        clearInterval(intervalId);
      });
    },
    eventHandlers: [
      {
        $el: $('a#btn_pitch_up'),
        event: 'mousedown',
        route: '/mousedown_btn_pitch_up',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_pitch_down'),
        event: 'mousedown',
        route: '/mousedown_btn_pitch_down',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_roll_left'),
        event: 'mousedown',
        route: '/mousedown_btn_roll_left',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_roll_right'),
        event: 'mousedown',
        route: '/mousedown_btn_roll_right',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_claw_open'),
        event: 'mousedown',
        route: '/mousedown_btn_claw_open',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_claw_close'),
        event: 'mousedown',
        route: '/mousedown_btn_claw_close',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_arm_up'),
        event: 'mousedown',
        route: '/mousedown_btn_arm_up',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_arm_down'),
        event: 'mousedown',
        route: '/mousedown_btn_arm_down',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_arm_left'),
        event: 'mousedown',
        route: '/mousedown_btn_arm_left',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_arm_right'),
        event: 'mousedown',
        route: '/mousedown_btn_arm_right',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_arm_back'),
        event: 'mousedown',
        route: '/mousedown_btn_arm_back',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_arm_forward'),
        event: 'mousedown',
        route: '/mousedown_btn_arm_forward',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_motor1_ccw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor1_ccw',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_motor1_cw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor1_cw',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_motor2_ccw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor2_ccw',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_motor2_cw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor2_cw',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_motor3_ccw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor3_ccw',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_motor3_cw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor3_cw',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_motor4_ccw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor4_ccw',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_motor4_cw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor4_cw',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_motor5_ccw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor5_ccw',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_motor5_cw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor5_cw',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_motor6_ccw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor6_ccw',
        handler: (data) => { },
      },
      {
        $el: $('a#btn_motor6_cw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor6_cw',
        handler: (data) => { },
      },
      {
        $el: $('button#ping-button'),
        event: 'mouseup',
        route: '/ping_rover',
        handler: (data) => {
            console.log(data);
            pingRover(data.ping_msg, data.ros_msg);
        },
    }
    ],
  };

  Site.init();

$("#mux-0").mouseup(function() {
    $.ajax({
        url: '/select_mux',
        type: 'POST',
        data: "0",
        success: function(response){
            $("button#mux").text("Device 0: Rover");
            appendToConsole(response.output);
            scrollToBottom();
        }
    })
})

$("#mux-1").mouseup(function() {
    $.ajax({
        url: '/select_mux',
        type: 'POST',
        data: "1",
        success: function(response){
            $("button#mux").text("Device 1: Arm");
            appendToConsole(response.output);
            scrollToBottom();
        }
    })
})

$("#mux-2").mouseup(function() {
    $.ajax({
        url: '/select_mux',
        type: 'POST',
        data: "2",
        success: function(response){
            $("button#mux").text("Device 2: Science");
            appendToConsole(response.output);
            scrollToBottom();
        }
    })
})

$("#mux-3").mouseup(function() {
    $.ajax({
        url: '/select_mux',
        type: 'POST',
        data: "3",
        success: function(response){
            $("button#mux").text("Device 3: Lidar");
            appendToConsole(response.output);
            scrollToBottom();
        }
    })
})

$("#send-serial-btn").mouseup(function() {
    let cmd = $("#serial-cmd-input").val();

    $.ajax({
        url: '/serial_cmd',
        type: 'POST',
        data: {
            'cmd': cmd
        },
        success: function(response){
            clearSerialCmd();
            appendToConsole(response.output);
            scrollToBottom();
        }
    })
})

$("#serial-cmd-input").on('keyup', function (e) {
    if (e.keyCode == 13) {
        let cmd = $("#serial-cmd-input").val();

        $.ajax({
            url: '/serial_cmd',
            type: 'POST',
            data: {
                'cmd': cmd
            },
            success: function(response){
                clearSerialCmd();
                appendToConsole(response.output);
                scrollToBottom();
            }
        })
    }
});

// update odroid rx data every second
setInterval(updateOdroidRx, 1000);


  // KEYBOARD EVENTS
  // rover ping
  document.addEventListener("keydown", function (event) {
    if (event.ctrlKey  &&  event.altKey  &&  event.code === "KeyP") {
        $.ajax("/ping_rover", {
             success: function(data) {
                 console.log(data);
                 pingRover(data.ping_msg, data.ros_msg);
             },
             error: function() {
                console.log("An error occured")
             }
          });
    }
  });

  // manual controls
  let $serialCmdInput = $("#serial-cmd-input");
  document.addEventListener("keydown", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyW") {
        toggleToManual();
        $("#click_btn_motor1_ccw > button").css("background-color", "rgb(255, 0, 0)");
    }
  });
  document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyW") {
        $("#click_btn_motor1_ccw > button").css("background-color", "rgb(74, 0, 0)");
    }
  });

  document.addEventListener("keydown", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyE") {
        toggleToManual();
        $("#click_btn_motor2_ccw > button").css("background-color", "rgb(255, 0, 0)");
    }
  });
  document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyE") {
        $("#click_btn_motor2_ccw > button").css("background-color", "rgb(74, 0, 0)");
    }
  });

  document.addEventListener("keydown", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyR") {
        toggleToManual();
        $("#click_btn_motor3_ccw > button").css("background-color", "rgb(255, 0, 0)");
    }
  });
  document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyR") {
        $("#click_btn_motor3_ccw > button").css("background-color", "rgb(74, 0, 0)");
    }
  });

  document.addEventListener("keydown", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyT") {
        toggleToManual();
        $("#click_btn_motor4_ccw > button").css("background-color", "rgb(255, 0, 0)");
    }
  });
  document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyT") {
        $("#click_btn_motor4_ccw > button").css("background-color", "rgb(74, 0, 0)");
    }
  });

  document.addEventListener("keydown", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyY") {
        toggleToManual();
        $("#click_btn_motor5_ccw > button").css("background-color", "rgb(255, 0, 0)");
    }
  });
  document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyY") {
        $("#click_btn_motor5_ccw > button").css("background-color", "rgb(74, 0, 0)");
    }
  });

  document.addEventListener("keydown", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyU") {
        toggleToManual();
        $("#click_btn_motor6_ccw > button").css("background-color", "rgb(255, 0, 0)");
    }
  });
  document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyU") {
        $("#click_btn_motor6_ccw > button").css("background-color", "rgb(74, 0, 0)");
    }
  });

  // automatic controls
  document.addEventListener("keydown", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyS") {
        toggleToManual();
        $("#click_btn_motor1_cw > button").css("background-color", "rgb(255, 0, 0)");
    }
  });
  document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyS") {
        $("#click_btn_motor1_cw > button").css("background-color", "rgb(74, 0, 0)");
    }
  });

  document.addEventListener("keydown", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyD") {
        toggleToManual();
        $("#click_btn_motor2_cw > button").css("background-color", "rgb(255, 0, 0)");
    }
  });
  document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyD") {
        $("#click_btn_motor2_cw > button").css("background-color", "rgb(74, 0, 0)");
    }
  });

  document.addEventListener("keydown", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyF") {
        toggleToManual();
        $("#click_btn_motor3_cw > button").css("background-color", "rgb(255, 0, 0)");
    }
  });
  document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyF") {
        $("#click_btn_motor3_cw > button").css("background-color", "rgb(74, 0, 0)");
    }
  });

  document.addEventListener("keydown", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyG") {
        toggleToManual();
        $("#click_btn_motor4_cw > button").css("background-color", "rgb(255, 0, 0)");
    }
  });
  document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyG") {
        $("#click_btn_motor4_cw > button").css("background-color", "rgb(74, 0, 0)");
    }
  });

  document.addEventListener("keydown", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyH") {
        toggleToManual();
        $("#click_btn_motor5_cw > button").css("background-color", "rgb(255, 0, 0)");
    }
  });
  document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyH") {
        $("#click_btn_motor5_cw > button").css("background-color", "rgb(74, 0, 0)");
    }
  });

  document.addEventListener("keydown", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyJ") {
        toggleToManual();
        $("#click_btn_motor6_cw > button").css("background-color", "rgb(255, 0, 0)");
    }
  });
  document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyJ") {
        $("#click_btn_motor6_cw > button").css("background-color", "rgb(74, 0, 0)");
    }
  });
});
