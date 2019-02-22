let mockArmTable = false;

// update console log
if (mockArmTable) {
    setInterval(mockArmTableLog, 1000);
}

//@TODO: fix implementation of odroid rx pub/sub to work on event triggers
// rather than through polling
// update odroid rx data every second
//setInterval(updateOdroidRx, 1000);


//@TODO: implement game loop for keyboard events:
// https://stackoverflow.com/questions/12273451/how-to-fix-delay-in-javascript-keydown

// KEYBOARD EVENTS
// rover ping
if (mockArmTable) {
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

    // motor 1
    document.addEventListener("keydown", function (event) {
        if (!$serialCmdInput.is(":focus") && event.code === "KeyW") {
            toggleToManual();
            $("#click_btn_motor1_ccw > button").css("background-color", "rgb(255, 0, 0)");

            if (mockArmTable) {
                let currentAngle = $("#m1-angle").text();

                if (currentAngle > -350) {
                    // simulate motor angles
                    $("#m1-angle").text(parseFloat(currentAngle) - 1);
                    // simulate motor currents
                    $("#m1-current").text("3.5");
                }
            }

        }
    });

    document.addEventListener("keydown", function (event) {
        if (!$serialCmdInput.is(":focus") && event.code === "KeyS") {
            toggleToManual();
            $("#click_btn_motor1_cw > button").css("background-color", "rgb(255, 0, 0)");

            if (mockArmTable) {
                let currentAngle = $("#m1-angle").text();

                if (currentAngle < 350) {
                    $("#m1-angle").text(parseFloat(currentAngle) + 1);
                    // simulate motor currents
                    $("#m1-current").text("3.5");
                }
            }
        }
    });

    // motor 2
    document.addEventListener("keydown", function (event) {
        if (!$serialCmdInput.is(":focus") && event.code === "KeyE") {
            toggleToManual();
            $("#click_btn_motor2_ccw > button").css("background-color", "rgb(255, 0, 0)");

            if (mockArmTable) {
                let currentAngle = $("#m2-angle").text();

                if (currentAngle > -75) {
                    $("#m2-angle").text(parseFloat(currentAngle) - 1);
                    // simulate motor currents
                    $("#m2-current").text("3.5");
                }
            }
        }
    });

    document.addEventListener("keydown", function (event) {
        if (!$serialCmdInput.is(":focus") && event.code === "KeyD") {
            toggleToManual();
            $("#click_btn_motor2_cw > button").css("background-color", "rgb(255, 0, 0)");

            if (mockArmTable) {
                let currentAngle = $("#m2-angle").text();

                if (currentAngle < 55) {
                    $("#m2-angle").text(parseFloat(currentAngle) + 1);
                    // simulate motor currents
                    $("#m2-current").text("3.5");
                }
            }
        }
    });

    // motor 3
    document.addEventListener("keydown", function (event) {
        if (!$serialCmdInput.is(":focus") && event.code === "KeyR") {
            toggleToManual();
            $("#click_btn_motor3_ccw > button").css("background-color", "rgb(255, 0, 0)");

            if (mockArmTable) {
                let currentAngle = $("#m3-angle").text();

                if (currentAngle > -155) {
                    $("#m3-angle").text(parseFloat(currentAngle) - 1);
                }
            }
        }
    });

    document.addEventListener("keydown", function (event) {
        if (!$serialCmdInput.is(":focus") && event.code === "KeyF") {
            toggleToManual();
            $("#click_btn_motor3_cw > button").css("background-color", "rgb(255, 0, 0)");

            if (mockArmTable) {
                let currentAngle = $("#m3-angle").text();

                if (currentAngle < 35) {
                    $("#m3-angle").text(parseFloat(currentAngle) + 1);
                }
            }
        }
    });

    // motor 4
    document.addEventListener("keydown", function (event) {
        if (!$serialCmdInput.is(":focus") && event.code === "KeyT") {
            toggleToManual();
            $("#click_btn_motor4_ccw > button").css("background-color", "rgb(255, 0, 0)");

            if (mockArmTable) {
                let currentAngle = $("#m4-angle").text();

                if (currentAngle > -55) {
                    $("#m4-angle").text(parseFloat(currentAngle) - 1);
                }
            }
        }
    });

    document.addEventListener("keydown", function (event) {
        if (!$serialCmdInput.is(":focus") && event.code === "KeyG") {
            toggleToManual();
            $("#click_btn_motor4_cw > button").css("background-color", "rgb(255, 0, 0)");

            if (mockArmTable) {
                // no angle limits
                let currentAngle = $("#m4-angle").text();

                if (currentAngle < 40) {
                    $("#m4-angle").text(parseFloat(currentAngle) + 1);
                }
            }
        }
    });

    // motor 5
    document.addEventListener("keydown", function (event) {
        if (!$serialCmdInput.is(":focus") && event.code === "KeyY") {
            toggleToManual();
            $("#click_btn_motor5_ccw > button").css("background-color", "rgb(255, 0, 0)");

            if (mockArmTable) {
                // no angle limits
                let currentAngle = $("#m5-angle").text();
                $("#m5-angle").text(parseFloat(currentAngle) - 1);
            }
        }
    });

    document.addEventListener("keydown", function (event) {
        if (!$serialCmdInput.is(":focus") && event.code === "KeyH") {
            toggleToManual();
            $("#click_btn_motor5_cw > button").css("background-color", "rgb(255, 0, 0)");

            if (mockArmTable) {
                // no angle limits
                let currentAngle = $("#m5-angle").text();
                $("#m5-angle").text(parseFloat(currentAngle) + 1);
            }
        }
    });

    // motor 6
    document.addEventListener("keydown", function (event) {
        if (!$serialCmdInput.is(":focus") && event.code === "KeyU") {
            toggleToManual();
            $("#click_btn_motor6_ccw > button").css("background-color", "rgb(255, 0, 0)");

            if (mockArmTable) {
                let currentAngle = $("#m6-angle").text();

                if (currentAngle > 0) {
                    $("#m6-angle").text(parseFloat(currentAngle) - 1);
                }
            }
        }
    });

    document.addEventListener("keydown", function (event) {
        if (!$serialCmdInput.is(":focus") && event.code === "KeyJ") {
            toggleToManual();
            $("#click_btn_motor6_cw > button").css("background-color", "rgb(255, 0, 0)");

            if (mockArmTable) {
                let currentAngle = $("#m6-angle").text();

                if (currentAngle < 75) {
                    $("#m6-angle").text(parseFloat(currentAngle) + 1);
                }
            }
        }
    });

} else {
    // Implement game loop
    var keyState = {};
    window.addEventListener('keydown',function(e){
        keyState[e.keyCode || e.which] = true;
    },true);
    window.addEventListener('keyup',function(e){
        keyState[e.keyCode || e.which] = false;
    },true);

    function gameLoop() {
        let $serialCmdInput = $("#serial-cmd-input");

        // 'w' --> m1 ccw
        if (!$serialCmdInput.is(":focus") && keyState[87]) {
            toggleToManual();
            $("#click_btn_motor1_ccw > button").css("background-color", "rgb(255, 0, 0)");

            $.ajax({
                url: '/manual_control',
                type: 'POST',
                data: {
                    cmd: 'w'
                },
                success: function(response){
                    appendToConsole("cmd: " + response.cmd);
                    scrollToBottom();
                }
            })
        }
        // 's' --> m1 cw
        if (!$serialCmdInput.is(":focus") && keyState[83]) {
            toggleToManual();
            $("#click_btn_motor1_cw > button").css("background-color", "rgb(255, 0, 0)");

            $.ajax({
                url: '/manual_control',
                type: 'POST',
                data: {
                    cmd: 's'
                },
                success: function(response){
                    appendToConsole("cmd: " + response.cmd);
                    scrollToBottom();
                }
            })
        }
        // 'e' --> m2 ccw
        if (!$serialCmdInput.is(":focus") && keyState[69]) {
            toggleToManual();
            $("#click_btn_motor2_ccw > button").css("background-color", "rgb(255, 0, 0)");

            $.ajax({
                url: '/manual_control',
                type: 'POST',
                data: {
                    cmd: 'e'
                },
                success: function(response){
                    appendToConsole("cmd: " + response.cmd);
                    scrollToBottom();
                }
            })
        }
        // 'd' --> m2 cw
        if (!$serialCmdInput.is(":focus") && keyState[68]) {
            toggleToManual();
            $("#click_btn_motor2_cw > button").css("background-color", "rgb(255, 0, 0)");

            $.ajax({
                url: '/manual_control',
                type: 'POST',
                data: {
                    cmd: 'd'
                },
                success: function(response){
                    appendToConsole("cmd: " + response.cmd);
                    scrollToBottom();
                }
            })
        }
        // 'r' --> m3 ccw
        if (!$serialCmdInput.is(":focus") && keyState[82]) {
            toggleToManual();
            $("#click_btn_motor3_ccw > button").css("background-color", "rgb(255, 0, 0)");

            $.ajax({
                url: '/manual_control',
                type: 'POST',
                data: {
                    cmd: 'r'
                },
                success: function(response){
                    appendToConsole("cmd: " + response.cmd);
                    scrollToBottom();
                }
            })
        }
        // 'f' --> m3 cw
        if (!$serialCmdInput.is(":focus") && keyState[70]) {
            toggleToManual();
            $("#click_btn_motor3_cw > button").css("background-color", "rgb(255, 0, 0)");

            $.ajax({
                url: '/manual_control',
                type: 'POST',
                data: {
                    cmd: 'f'
                },
                success: function(response){
                    appendToConsole("cmd: " + response.cmd);
                    scrollToBottom();
                }
            })
        }
        // 't' --> m4 ccw
        if (!$serialCmdInput.is(":focus") && keyState[84]) {
            toggleToManual();
            $("#click_btn_motor4_ccw > button").css("background-color", "rgb(255, 0, 0)");

            $.ajax({
                url: '/manual_control',
                type: 'POST',
                data: {
                    cmd: 't'
                },
                success: function(response){
                    appendToConsole("cmd: " + response.cmd);
                    scrollToBottom();
                }
            })
        }
        // 'g' --> m4 cw
        if (!$serialCmdInput.is(":focus") && keyState[71]) {
            toggleToManual();
            $("#click_btn_motor4_cw > button").css("background-color", "rgb(255, 0, 0)");

            $.ajax({
                url: '/manual_control',
                type: 'POST',
                data: {
                    cmd: 'g'
                },
                success: function(response){
                    appendToConsole("cmd: " + response.cmd);
                    scrollToBottom();
                }
            })
        }
        // 'y' --> m5 ccw
        if (!$serialCmdInput.is(":focus") && keyState[89]) {
            toggleToManual();
            $("#click_btn_motor5_ccw > button").css("background-color", "rgb(255, 0, 0)");

            $.ajax({
                url: '/manual_control',
                type: 'POST',
                data: {
                    cmd: 'y'
                },
                success: function(response){
                    appendToConsole("cmd: " + response.cmd);
                    scrollToBottom();
                }
            })
        }
        // 'h' --> m5 cw
        if (!$serialCmdInput.is(":focus") && keyState[72]) {
            toggleToManual();
            $("#click_btn_motor5_cw > button").css("background-color", "rgb(255, 0, 0)");

            $.ajax({
                url: '/manual_control',
                type: 'POST',
                data: {
                    cmd: 'h'
                },
                success: function(response){
                    appendToConsole("cmd: " + response.cmd);
                    scrollToBottom();
                }
            })
        }
        // 'u' --> m6 ccw
        if (!$serialCmdInput.is(":focus") && keyState[85]) {
            toggleToManual();
            $("#click_btn_motor6_ccw > button").css("background-color", "rgb(255, 0, 0)");

            $.ajax({
                url: '/manual_control',
                type: 'POST',
                data: {
                    cmd: 'u'
                },
                success: function(response){
                    appendToConsole("cmd: " + response.cmd);
                    scrollToBottom();
                }
            })
        }
        // 'j' --> m6 cw
        if (!$serialCmdInput.is(":focus") && keyState[74]) {
            toggleToManual();
            $("#click_btn_motor6_cw > button").css("background-color", "rgb(255, 0, 0)");

            $.ajax({
                url: '/manual_control',
                type: 'POST',
                data: {
                    cmd: 'j'
                },
                success: function(response){
                    appendToConsole("cmd: " + response.cmd);
                    scrollToBottom();
                }
            })
        }

        // redraw/reposition your object here
        // also redraw/animate any objects not controlled by the user

        setTimeout(gameLoop, 10);
    }
    gameLoop();
}

// In any case
let $serialCmdInput = $("#serial-cmd-input");

document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyW") {
        $("#click_btn_motor1_ccw > button").css("background-color", "rgb(74, 0, 0)");

        if (mockArmTable) {
            $("#m1-current").text("0.2");
        }
    }
});

document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyS") {
        $("#click_btn_motor1_cw > button").css("background-color", "rgb(74, 0, 0)");
        $("#m1-current").text("0.2");
    }
});

document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyE") {
        $("#click_btn_motor2_ccw > button").css("background-color", "rgb(74, 0, 0)");
        $("#m2-current").text("0.2");
    }
});

document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyD") {
        $("#click_btn_motor2_cw > button").css("background-color", "rgb(74, 0, 0)");
        $("#m2-current").text("0.2");
    }
});

document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyR") {
        $("#click_btn_motor3_ccw > button").css("background-color", "rgb(74, 0, 0)");
    }
});

document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyF") {
        $("#click_btn_motor3_cw > button").css("background-color", "rgb(74, 0, 0)");
    }
});

document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyT") {
        $("#click_btn_motor4_ccw > button").css("background-color", "rgb(74, 0, 0)");
    }
});

document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyG") {
        $("#click_btn_motor4_cw > button").css("background-color", "rgb(74, 0, 0)");
    }
});

document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyY") {
        $("#click_btn_motor5_ccw > button").css("background-color", "rgb(74, 0, 0)");
    }
});

document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyH") {
        $("#click_btn_motor5_cw > button").css("background-color", "rgb(74, 0, 0)");
    }
});

document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyU") {
        $("#click_btn_motor6_ccw > button").css("background-color", "rgb(74, 0, 0)");
    }
});

document.addEventListener("keyup", function (event) {
    if (!$serialCmdInput.is(":focus") && event.code === "KeyJ") {
        $("#click_btn_motor6_cw > button").css("background-color", "rgb(74, 0, 0)");
    }
});
