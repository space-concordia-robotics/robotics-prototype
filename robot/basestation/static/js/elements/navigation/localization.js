/*
This script uses a lot of p5.js functions and concepts.
I recommend using the reference: https://p5js.org/reference/#/p5/
before doing any work on this script as it will greatly facilitate the process.
*/




$(document).ready(() => {
    roverHeading = 0
    roverX = 0
    roverY = 0
    zoomValue = 1
    keys = { //javascript keycodes
        'r': 82,
        'w': 87,
        'a': 65,
        's': 83,
        'd': 68
    }
    PIXELS_PER_METER = 5
    baseLat = $('#antenna-latitude').text()
    baseLong = $('#antenna-longitude').text()
    MAX_ZOOM = 2
    MIN_ZOOM = 0.15
    CANVAS_HEIGHT = 300
    CANVAS_WIDTH = 450

    rover_position_listener.subscribe(function(message) {
        roverLat = message.x
        roverLong = message.y
        roverHeading = message.z
    })



    function setup() {
        canvas = createCanvas(CANVAS_WIDTH, CANVAS_HEIGHT)
        canvas.parent('#canvasContainer')
        canvasPosition = $('#defaultCanvas0').offset()
        angleMode(DEGREES) // makes p5.js use DEG instead of RAD
    }

    function draw() {
        // all of the manual controls
        if (keyIsDown(keys['r'])) { //reset position
            roverHeading = 0;
            zoomValue = 1;
            roverX = roverY = 0;
        }
        if (keyIsDown(keys['a'])) { // spin ccw
            roverHeading -= 1
        }
        if (keyIsDown(keys['d'])) { // spin cw
            roverHeading += 1
        }
        if (keyIsDown(keys['w'])) { // move forward
            roverX += sin(roverHeading);
            roverY -= cos(roverHeading);
        }
        if (keyIsDown(keys['s'])) { // move backward
            roverX -= sin(roverHeading);
            roverY += cos(roverHeading);
        }

        background(225)
        translate(width / 2, height / 2)

        scale(zoomValue)
        drawGrid()
        // (x,y,diameter)
        circle(0, 0, 10)
        drawRover()
    }

    // function to draw the lines and values on canvas
    // this grid can be drawn of any size, doesn't have to follow canvas values
    // i.e. we can make it so that 10px = 1m and make the grid go from -1km to 1km (-10000 to 10000 pixels)
    // shouldn't forget to adjust the scaling min/max so the whole grid can fit into the small canvas if needed
    function drawGrid(message) {
        stroke(200)
        fill(120)
        for (let x = -10000; x < 10000; x += 100) {
            line(x, -10000, x, 10000)
            text(x, x + 1, 12)
        }
        for (let y = -10000; y < 10000; y += 100) {
            line(-10000, y, 10000, y)
            text(y, 1, y + 12)
        }
    }

    function drawRover() {
        //checkRoverPosition()
        push()
        translate(roverX, roverY)
        rotate(roverHeading)
        // 3 points (x, y, x, y, x ,y)
        triangle(0, -15, 10, 10, -10, 10)
        pop()
    }

    // register mouse wheel event and do the zoom
    function mouseWheel(e) {
        if (winMouseX > canvasPosition['left'] && winMouseX < canvasPosition['left'] +
            width && winMouseY > canvasPosition['top'] && winMouseY < canvasPosition['top'] + height) {
            // delta is too big for the scale() function, so must be divided by 500 to reduce amount of zoom per scroll
            let newZoomValue = zoomValue - e.delta / 500
            if (newZoomValue >= MIN_ZOOM && newZoomValue <= MAX_ZOOM) {
                zoomValue = newZoomValue
                return false
            }
        }
    }

    // function that gets the data from the rover_position topic
    function checkRoverPosition() {
        let roverLat = $('#rover-latitude').text()
        let roverLong = $('#rover-longitude').text()
        roverHeading = $('#rover-heading').text()

        adjustValuesToCartesian(roverLat, roverLong, roverHeading)
    }

    // function which converts the values into something usable by the canvas
    function adjustValuesToCartesian(roverLat, roverLong, roverHeading) {
        let direction = directionToRover(baseLat, baseLong, roverLat, roverLong) // angle in degrees
        let distance = distanceToRover(baseLat, baseLong, roverLat, roverLong) // distance in meters

        roverX = PIXELS_PER_METER * distance * Math.cos(direction)
        roverY = PIXELS_PER_METER * distance * Math.sin(direction)
    }
})
