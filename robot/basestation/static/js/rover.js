//@TODO: implement game loop for keyboard events:
// https://stackoverflow.com/questions/12273451/how-to-fix-delay-in-javascript-keydown

let mockRoverTable = true;

// rover drive keyboard events
$(document).keydown(function(e) {
  switch(e.which) {
      case 37: // left
          $("#rover-left > button").css("background-color", "rgb(255, 0, 0)");
          if (mockRoverTable) {
              $("#left-front-rpm").text("35");
              $("#left-front-current").text("3.5");
              $("#left-mid-rpm").text("35");
              $("#left-mid-current").text("3.5");
              $("#left-rear-rpm").text("35");
              $("#left-rear-current").text("3.5");
              $("#right-front-rpm").text("35");
              $("#right-front-current").text("3.5");
              $("#right-mid-rpm").text("35");
              $("#right-mid-current").text("3.5");
              $("#right-rear-rpm").text("35");
              $("#right-rear-current").text("3.5");
          }
          break;

      case 38: // up
          $("#rover-up > button").css("background-color", "rgb(255, 0, 0)");
          if (mockRoverTable) {
              $("#left-front-rpm").text("35");
              $("#left-front-current").text("3.5");
              $("#left-mid-rpm").text("35");
              $("#left-mid-current").text("3.5");
              $("#left-rear-rpm").text("35");
              $("#left-rear-current").text("3.5");
              $("#right-front-rpm").text("35");
              $("#right-front-current").text("3.5");
              $("#right-mid-rpm").text("35");
              $("#right-mid-current").text("3.5");
              $("#right-rear-rpm").text("35");
              $("#right-rear-current").text("3.5");
          }
          break;

      case 39: // right
          $("#rover-right > button").css("background-color", "rgb(255, 0, 0)");
          if (mockRoverTable) {
              $("#left-front-rpm").text("35");
              $("#left-front-current").text("3.5");
              $("#left-mid-rpm").text("35");
              $("#left-mid-current").text("3.5");
              $("#left-rear-rpm").text("35");
              $("#left-rear-current").text("3.5");
              $("#right-front-rpm").text("35");
              $("#right-front-current").text("3.5");
              $("#right-mid-rpm").text("35");
              $("#right-mid-current").text("3.5");
              $("#right-rear-rpm").text("35");
              $("#right-rear-current").text("3.5");
          }
          break;

      case 40: // down
          $("#rover-down > button").css("background-color", "rgb(255, 0, 0)");
          if (mockRoverTable) {
              $("#left-front-rpm").text("35");
              $("#left-front-current").text("3.5");
              $("#left-mid-rpm").text("35");
              $("#left-mid-current").text("3.5");
              $("#left-rear-rpm").text("35");
              $("#left-rear-current").text("3.5");
              $("#right-front-rpm").text("35");
              $("#right-front-current").text("3.5");
              $("#right-mid-rpm").text("35");
              $("#right-mid-current").text("3.5");
              $("#right-rear-rpm").text("35");
              $("#right-rear-current").text("3.5");
          }
          break;

      default: return; // exit this handler for other keys
  }
  e.preventDefault(); // prevent the default action (scroll / move caret)
});

$(document).keyup(function(e) {
  switch(e.which) {
      case 37: // left
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

      case 38: // up
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

      case 39: // right
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

      case 40: // down
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

if (mockRoverTable) {
    setInterval(mockRoverTableLog, 1000);
}
