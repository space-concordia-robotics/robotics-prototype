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
