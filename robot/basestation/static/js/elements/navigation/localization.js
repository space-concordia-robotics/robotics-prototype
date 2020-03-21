/*
This script uses a lot of p5.js functions and concepts.
I recommend using the reference: https://p5js.org/reference/#/p5/
before doing any work on this script as it will greatly facilitate the process.
*/

$(document).ready(() => {
    myp5 = new p5(sketch)
})

let sketch = function(p) {
    roverHeading = 0
    roverX = 0 // cartesian coordinates of rover in pixels
    roverY = 0
    baseLat = null
    baseLong = null
    rover_position_listener.subscribe(function(message) {
        roverLong = message.x
        roverLat = message.y
        roverHeading = message.z
        initializeBase(message.x, message.y)
        adjustValuesToCartesian(roverLat, roverLong)
    })

    dragDiffX = 0
    dragDiffY = 0
    zoomValue = 1

    r = 82 // JS keycode
    toggle = false

    MAX_ZOOM = 2
    MIN_ZOOM = 0.15
    CANVAS_HEIGHT = 300
    CANVAS_WIDTH = 450


    p.setup = function() {
        canvas = p.createCanvas(CANVAS_WIDTH, CANVAS_HEIGHT)
        canvas.parent('#canvasContainer')
        p.angleMode(p.DEGREES) // makes p5.js use DEG instead of RAD
    }

    p.draw = function() {
        if (p.keyIsPressed) {
            controls()
        }

        p.background(225)
        drawResizeButton()
        p.translate(p.width / 2 + dragDiffX, p.height / 2 + dragDiffY)
        p.scale(zoomValue)
        p.stroke(200)
        p.fill(120)
        drawGrid()
        drawBase()
        drawRover()
    }

    /* ----------------------------------------
    FUNCTIONS THAT DRAW THINGS
    */

    function drawBase() {
        p.push()
        p.scale(1 / zoomValue)
        p.circle(0, 0, 10) // (x, y, diameter)
        p.pop()
    }

    function drawRover() {
        p.push()
        p.translate(roverX, roverY)
        p.rotate(roverHeading)
        p.scale(1 / zoomValue)
        p.triangle(0, -15, 10, 10, -10, 10) // 3 points (x, y, x, y, x ,y)
        p.pop()
    }

    function drawGrid() {
        // draw a line every 100 pixels for 1000 pixels
        for (let x = -1000; x < 1000; x += 100) {
            p.line(x, -1000, x, 1000)
            p.text(x, x + 1, 12)
        }

        for (let y = -1000; y < 1000; y += 100) {
            p.line(-1000, y, 1000, y)
            p.text(y, 1, y + 12)
        }
    }

    function drawResizeButton() {
        p.push()
        if (toggle) {
            p.fill(0, 250, 0) // rgb
        } else {
            p.fill(250, 0, 0) // rgb
        }
        p.arc(p.width, p.height, 20, 20, 180, 270, p.PIE) // (x, y, width, height, start, stop, mode)
        p.pop()
    }


    /* ----------------------------------------
    p5 EVENT FUNCTIONS
    */

    // register mouse wheel event and zoom accordingly
    p.mouseWheel = function(e) {
        if (p.mouseX > 0 && p.mouseX < p.width && p.mouseY > 0 && p.mouseY < p.height) {
            let newZoomValue = zoomValue - e.delta / 500
            if (newZoomValue >= MIN_ZOOM && newZoomValue <= MAX_ZOOM) {
                zoomValue = newZoomValue
            }
            return false
        }
    }

    // drag canvas view
    p.mouseDragged = function() {
        if (p.mouseX > 0 && p.mouseX < p.width - 10 && p.mouseY > 0 && p.mouseY < p.height - 10) {
            dragDiffX += p.movedX
            dragDiffY += p.movedY
        }
    }

    // resize canvas
    p.mouseClicked = function() {
        let r = 10 // button radius
        let d = p.dist(p.mouseX, p.mouseY, p.width, p.height) // distance between mouse and lower right corner

        if (d < r) {
            toggle = !toggle
        }

        if (toggle) {
            p.resizeCanvas(p.mouseX, p.mouseY)
        }
    }


    /* ----------------------------------------
    UTILITY FUNCTIONS
    */

    // function which converts the values into something usable by the canvas
    function adjustValuesToCartesian(roverLat, roverLong) {
        let direction = directionToRover(baseLat, baseLong, roverLat, roverLong) // angle in degrees
        let distance = distanceToRover(baseLat, baseLong, roverLat, roverLong) // distance in meters
        let directionInRad = direction * Math.PI / 180 // convert for Math in JS

        roverX = distance * Math.sin(directionInRad)
        roverY = -distance * Math.cos(directionInRad)
        console.log('distance: ' + distance)
        console.log('direction: ' + direction)
        console.log('x: ' + roverX)
        console.log('y: ' + roverY)
    }

    // implements manual controls.
    function controls() {
        if (p.keyIsDown(r)) { //reset position
            zoomValue = 1
            dragDiffX = dragDiffY = 0
            p.resizeCanvas(CANVAS_WIDTH, CANVAS_HEIGHT)
        }
        if (p.keyIsDown(p.LEFT_ARROW)) { // spin ccw
            roverHeading -= 1
        }
        if (p.keyIsDown(p.RIGHT_ARROW)) { // spin cw
            roverHeading += 1
        }
        if (p.keyIsDown(p.UP_ARROW)) { // move forward
            roverX += p.sin(roverHeading)
            roverY -= p.cos(roverHeading)
        }
        if (p.keyIsDown(p.DOWN_ARROW)) { // move backward
            roverX -= p.sin(roverHeading)
            roverY += p.cos(roverHeading)
        }
    }

    function initializeBase(x, y) {
        if (baseLat == null && baseLong == null) {
            baseLong = x
            baseLat = y
        }
    }
}
