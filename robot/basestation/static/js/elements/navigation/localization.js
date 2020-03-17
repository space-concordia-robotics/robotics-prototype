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
    dragDiffX = 0
    dragDiffY = 0
    zoomValue = 1
    keys = { //javascript keycodes
        'r': 82,
        'w': 87,
        'a': 65,
        's': 83,
        'd': 68
    }
    toggle = false

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
        sketch.angleMode(sketch.DEGREES) // makes p5.js use DEG instead of RAD
    }

    sketch.draw = function() {

        if (sketch.keyIsPressed == true) {
            controls()
        }

        sketch.background(225)
        drawResizeButton()
        sketch.translate(sketch.width / 2 + dragDiffX, sketch.height / 2 + dragDiffY)
        sketch.scale(zoomValue)
        drawGrid()
        // (x, y, diameter)
        sketch.circle(0, 0, 10)
        drawRover()
    }

    /* ----------------------------------------
    FUNCTIONS THAT DRAW THINGS
    */

    function drawRover() {
        sketch.push()
        sketch.translate(roverX, roverY)
        sketch.rotate(roverHeading)
        // 3 points (x, y, x, y, x ,y)
        sketch.triangle(0, -15, 10, 10, -10, 10)
        sketch.pop()
    }

    function drawGrid() {
        sketch.push()
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

        sketch.pop()
    }

    function drawResizeButton() {
        sketch.push()
        if (toggle) {
            sketch.fill(0, 250, 0) // rgb
        } else {
            sketch.fill(250, 0, 0) // rgb
        }
        sketch.arc(sketch.width, sketch.height, 20, 20, 180, 270, sketch.PIE) // (x, y, width, height, start, stop, mode)
        sketch.pop()
    }


    /* ----------------------------------------
    p5 EVENT FUNCTIONS
    */

    // register mouse wheel event and zoom accordingly
    sketch.mouseWheel = function(e) {
        if (sketch.mouseX > 0 && sketch.mouseX < sketch.width && sketch.mouseY > 0 && sketch.mouseY < sketch.height) {
            let newZoomValue = zoomValue - e.delta / 500
            if (newZoomValue >= MIN_ZOOM && newZoomValue <= MAX_ZOOM) {
                zoomValue = newZoomValue

            }
            return false
        }
    }

    // drag canvas view
    sketch.mouseDragged = function() {
        if (sketch.mouseX > 0 && sketch.mouseX < sketch.width - 10 && sketch.mouseY > 0 && sketch.mouseY < sketch.height - 10) {
            dragDiffX += sketch.movedX
            dragDiffY += sketch.movedY
        }
    }

    // resize canvas
    sketch.mouseClicked = function() {
        let r = 10 // button radius
        let d = sketch.dist(sketch.mouseX, sketch.mouseY, sketch.width, sketch.height)

        if (d < r) {
            toggle = !toggle
        }

        if (toggle) {
            sketch.resizeCanvas(sketch.mouseX, sketch.mouseY)
        }
        return false
    }


    /* ----------------------------------------
    UTILITY FUNCTIONS
    */

    // function which converts the values into something usable by the canvas
    function adjustValuesToCartesian(roverLat, roverLong) {
        let direction = directionToRover(baseLat, baseLong, roverLat, roverLong) // angle in degrees
        let distance = distanceToRover(baseLat, baseLong, roverLat, roverLong) // distance in meters

        roverX = distance * Math.cos(direction)
        roverY = distance * Math.sin(direction)
    }

    // implements manual controls.
    function controls() {
        if (sketch.keyIsDown(keys['r'])) { //reset position
            roverHeading = 0
            zoomValue = 1
            roverX = roverY = 0
            dragDiffX = dragDiffY = 0
            sketch.resizeCanvas(CANVAS_WIDTH, CANVAS_HEIGHT)
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
    }
}
