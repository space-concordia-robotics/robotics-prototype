/*
This script uses a lot of p5.js functions and concepts.
I recommend using the reference: https://p5js.org/reference/#/p5/
before doing any work on this script as it will greatly facilitate the process.
*/




$(document).ready(() => {
    myp5 = new p5(sketch)
})

let sketch = function(sketch) {
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

    MAX_ZOOM = 2
    MIN_ZOOM = 0.15
    CANVAS_HEIGHT = 300
    CANVAS_WIDTH = 450
    baseLat = 0
    baseLong = 0
    rover_position_listener.subscribe(function(message) {
        roverLat = message.x
        roverLong = message.y
        roverHeading = message.z
        adjustValuesToCartesian(roverLat, roverLong)
    })



    sketch.setup = function() {
        canvas = sketch.createCanvas(CANVAS_WIDTH, CANVAS_HEIGHT)
        canvas.parent('#canvasContainer')
        canvasPosition = $('#defaultCanvas0').offset()
        sketch.angleMode(sketch.DEGREES) // makes p5.js use DEG instead of RAD
    }

    sketch.draw = function() {
        // all of the manual controls
        if (sketch.keyIsDown(keys['r'])) { //reset position
            roverHeading = 0
            zoomValue = 1
            roverX = roverY = 0
        }
        if (sketch.keyIsDown(sketch.LEFT_ARROW)) { // spin ccw
            roverHeading -= 1
        }
        if (sketch.keyIsDown(sketch.RIGHT_ARROW)) { // spin cw
            roverHeading += 1
        }
        if (sketch.keyIsDown(sketch.UP_ARROW)) { // move forward
            roverX += sketch.sin(roverHeading)
            roverY -= sketch.cos(roverHeading)
        }
        if (sketch.keyIsDown(sketch.DOWN_ARROW)) { // move backward
            roverX -= sketch.sin(roverHeading)
            roverY += sketch.cos(roverHeading)
        }

        sketch.background(225)
        sketch.translate(sketch.width / 2, sketch.height / 2)

        sketch.scale(zoomValue)
        drawGrid()
        // (x,y,diameter)
        sketch.circle(0, 0, 10)
        drawRover()
    }

    // function to draw the lines and values on canvas
    // this grid can be drawn of any size, doesn't have to follow canvas values
    // i.e. we can make it so that 10px = 1m and make the grid go from -1km to 1km (-10000 to 10000 pixels)
    // shouldn't forget to adjust the scaling min/max so the whole grid can fit into the small canvas if needed
    function drawGrid(message) {
        sketch.stroke(200)
        sketch.fill(120)
        for (let x = -10000; x < 10000; x += 100) {
            sketch.line(x, -10000, x, 10000)
            sketch.text(x, x + 1, 12)
        }
        for (let y = -10000; y < 10000; y += 100) {
            sketch.line(-10000, y, 10000, y)
            sketch.text(y, 1, y + 12)
        }
    }

    function drawRover() {
        sketch.push()
        sketch.translate(roverX, roverY)
        sketch.rotate(roverHeading)
        // 3 points (x, y, x, y, x ,y)
        sketch.triangle(0, -15, 10, 10, -10, 10)
        sketch.pop()
    }

    // register mouse wheel event and do the zoom
    function mouseWheel(e) {
        if (sketch.winMouseX > canvasPosition['left'] && sketch.winMouseX < canvasPosition['left'] +
            sketch.width && sketch.winMouseY > canvasPosition['top'] && sketch.winMouseY < canvasPosition['top'] + sketch.height) {
            // delta is too big for the scale() function, so must be divided by 500 to reduce amount of zoom per scroll
            let newZoomValue = zoomValue - e.delta / 500
            if (newZoomValue >= MIN_ZOOM && newZoomValue <= MAX_ZOOM) {
                zoomValue = newZoomValue
                return false
            }
        }
    }


    // function which converts the values into something usable by the canvas
    function adjustValuesToCartesian(roverLat, roverLong) {
        let direction = directionToRover(baseLat, baseLong, roverLat, roverLong) // angle in degrees
        let distance = distanceToRover(baseLat, baseLong, roverLat, roverLong) // distance in meters

        roverX = distance * Math.cos(direction)
        roverY = distance * Math.sin(direction)
    }
}
