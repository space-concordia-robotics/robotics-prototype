//@TODO: implement game loop for keyboard events:
// https://stackoverflow.com/questions/12273451/how-to-fix-delay-in-javascript-keydown

let mockRoverLog = true;

// Rover log
function appendMockToRoverLog() {
    $("#rover-log").append("All motors alive.\n");
    $("#rover-log").scrollTop($("rover-log")[0].scrollHeight);
}

// rover drive keyboard events
$(document).keydown(function(e) {
  switch(e.which) {
      case 37: // left
      $("#rover-left > button").css("background-color", "rgb(255, 0, 0)");
      break;

      case 38: // up
      $("#rover-up > button").css("background-color", "rgb(255, 0, 0)");
      break;

      case 39: // right
      $("#rover-right > button").css("background-color", "rgb(255, 0, 0)");
      break;

      case 40: // down
      $("#rover-down > button").css("background-color", "rgb(255, 0, 0)");
      break;

      default: return; // exit this handler for other keys
  }
  e.preventDefault(); // prevent the default action (scroll / move caret)
});

$(document).keyup(function(e) {
  switch(e.which) {
      case 37: // left
      $("#rover-left > button").css("background-color", "rgb(74, 0, 0)");
      break;

      case 38: // up
      $("#rover-up > button").css("background-color", "rgb(74, 0, 0)");
      break;

      case 39: // right
      $("#rover-right > button").css("background-color", "rgb(74, 0, 0)");
      break;

      case 40: // down
      $("#rover-down > button").css("background-color", "rgb(74, 0, 0)");
      break;

      default: return; // exit this handler for other keys
  }
  e.preventDefault(); // prevent the default action (scroll / move caret)
});

if (mockRoverLog) {
    setInterval(appendMockToRoverLog, 1000);
}
