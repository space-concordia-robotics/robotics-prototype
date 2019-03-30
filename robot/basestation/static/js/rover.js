//@TODO: implement game loop for keyboard events:
// https://stackoverflow.com/questions/12273451/how-to-fix-delay-in-javascript-keydown

let mockRoverTable = false;

$(document).keyup(function(e) {
  switch(e.which) {
      case 65: // left
          $("#rover-left > button").css("background-color", "rgb(74, 0, 0)");

          if (mockRoverTable) {
              $("#left-front-rpm").text("0");
              $("#left-front-current").text("0.3");
              $("#left-mid-rpm").text("0");
              $("#left-mid-current").text("0.3");
              $("#left-rear-rpm").text("0");
              $("#left-rear-current").text("0.3");
              $("#right-front-rpm").text("0");
              $("#right-front-current").text("0.3");
              $("#right-mid-rpm").text("0");
              $("#right-mid-current").text("0.3");
              $("#right-rear-rpm").text("0");
              $("#right-rear-current").text("0.3");
          }
          break;

      case 87: // up
          $("#rover-up > button").css("background-color", "rgb(74, 0, 0)");
          if (mockRoverTable) {
              $("#left-front-rpm").text("0");
              $("#left-front-current").text("0.3");
              $("#left-mid-rpm").text("0");
              $("#left-mid-current").text("0.3");
              $("#left-rear-rpm").text("0");
              $("#left-rear-current").text("0.3");
              $("#right-front-rpm").text("0");
              $("#right-front-current").text("0.3");
              $("#right-mid-rpm").text("0");
              $("#right-mid-current").text("0.3");
              $("#right-rear-rpm").text("0");
              $("#right-rear-current").text("0.3");
          }
          break;

      case 68: // right
          $("#rover-right > button").css("background-color", "rgb(74, 0, 0)");
          if (mockRoverTable) {
              $("#left-front-rpm").text("0");
              $("#left-front-current").text("0.3");
              $("#left-mid-rpm").text("0");
              $("#left-mid-current").text("0.3");
              $("#left-rear-rpm").text("0");
              $("#left-rear-current").text("0.3");
              $("#right-front-rpm").text("0");
              $("#right-front-current").text("0.3");
              $("#right-mid-rpm").text("0");
              $("#right-mid-current").text("0.3");
              $("#right-rear-rpm").text("0");
              $("#right-rear-current").text("0.3");
          }
          break;

      case 83: // down
          $("#rover-down > button").css("background-color", "rgb(74, 0, 0)");
          if (mockRoverTable) {
              $("#left-front-rpm").text("0");
              $("#left-front-current").text("0.3");
              $("#left-mid-rpm").text("0");
              $("#left-mid-current").text("0.3");
              $("#left-rear-rpm").text("0");
              $("#left-rear-current").text("0.3");
              $("#right-front-rpm").text("0");
              $("#right-front-current").text("0.3");
              $("#right-mid-rpm").text("0");
              $("#right-mid-current").text("0.3");
              $("#right-rear-rpm").text("0");
              $("#right-rear-current").text("0.3");
          }
          break;

      default: return; // exit this handler for other keys
  }
  e.preventDefault(); // prevent the default action (scroll / move caret)
});

// GAME LOOP CONTROL

var keyState = {};
window.addEventListener('keydown',function(e){
    keyState[e.keyCode || e.which] = true;
},true);
window.addEventListener('keyup',function(e){
    keyState[e.keyCode || e.which] = false;
},true);

function gameLoop() {
    // 'a' --> rover turn left
    if (keyState[65]) {

        $("#rover-left > button").css("background-color", "rgb(255, 0, 0)");

        if (mockRoverTable) {
            $("#left-front-rpm").text("0");
            $("#left-front-current").text("0.3");
            $("#left-mid-rpm").text("0");
            $("#left-mid-current").text("0.3");
            $("#left-rear-rpm").text("0");
            $("#left-rear-current").text("0.3");
            $("#right-front-rpm").text("0");
            $("#right-front-current").text("0.3");
            $("#right-mid-rpm").text("0");
            $("#right-mid-current").text("0.3");
            $("#right-rear-rpm").text("0");
            $("#right-rear-current").text("0.3");
        }

        $.ajax({
            url: '/rover_drive',
            type: 'POST',
            data: {
                cmd: 'a'
            },
            success: function(response){
                appendToConsole("cmd: " + response.cmd);
                scrollToBottom();
            }
        })
    }
    // 'w' --> rover forward
    if (keyState[87]) {

        $("#rover-up > button").css("background-color", "rgb(255, 0, 0)");

        if (mockRoverTable) {
            $("#left-front-rpm").text("0");
            $("#left-front-current").text("0.3");
            $("#left-mid-rpm").text("0");
            $("#left-mid-current").text("0.3");
            $("#left-rear-rpm").text("0");
            $("#left-rear-current").text("0.3");
            $("#right-front-rpm").text("0");
            $("#right-front-current").text("0.3");
            $("#right-mid-rpm").text("0");
            $("#right-mid-current").text("0.3");
            $("#right-rear-rpm").text("0");
            $("#right-rear-current").text("0.3");
        }

        $.ajax({
            url: '/rover_drive',
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
    // 'd' --> rover right
    if (keyState[68]) {

        $("#rover-right > button").css("background-color", "rgb(255, 0, 0)");

        if (mockRoverTable) {
            $("#left-front-rpm").text("0");
            $("#left-front-current").text("0.3");
            $("#left-mid-rpm").text("0");
            $("#left-mid-current").text("0.3");
            $("#left-rear-rpm").text("0");
            $("#left-rear-current").text("0.3");
            $("#right-front-rpm").text("0");
            $("#right-front-current").text("0.3");
            $("#right-mid-rpm").text("0");
            $("#right-mid-current").text("0.3");
            $("#right-rear-rpm").text("0");
            $("#right-rear-current").text("0.3");
        }

        $.ajax({
            url: '/rover_drive',
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

    // 's' --> rover back
    if (keyState[83]) {

        $("#rover-down > button").css("background-color", "rgb(255, 0, 0)");

        if (mockRoverTable) {
            $("#left-front-rpm").text("0");
            $("#left-front-current").text("0.3");
            $("#left-mid-rpm").text("0");
            $("#left-mid-current").text("0.3");
            $("#left-rear-rpm").text("0");
            $("#left-rear-current").text("0.3");
            $("#right-front-rpm").text("0");
            $("#right-front-current").text("0.3");
            $("#right-mid-rpm").text("0");
            $("#right-mid-current").text("0.3");
            $("#right-rear-rpm").text("0");
            $("#right-rear-current").text("0.3");
        }

        $.ajax({
            url: '/rover_drive',
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
    // redraw/reposition your object here
    // also redraw/animate any objects not controlled by the user

    setTimeout(gameLoop, 10);
}

gameLoop();

if (mockRoverTable) {
    setInterval(mockRoverTableLog, 1000);
}
