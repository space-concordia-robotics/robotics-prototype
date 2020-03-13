//$(document).ready(() => {
let roverHeading = 0
let roverX = 0
let roverY = 0
let zoomValue = 1
let MAX_ZOOM = 2
let MIN_ZOOM = 0.25
let canvasPosition
let keys = { //javascript keycodes
    'r': 82,
    'w': 87,
    'a': 65,
    's': 83,
    'd': 68
}
let PIXELS_PER_METER = 5
let baseLat = $('#antenna-latitude').text()
let baseLong = $('#antenna-longitude').text()
let CANVAS_HEIGHT = 300
let CANVAS_WIDTH = 450

let BASE_POSITION = {
    'x': 0,
    'y': 0
}
let BASE_DIAMETER = 10

let ROVER_POINT_A = {
    'x': 0,
    'y': -15
}
let ROVER_POINT_B = {
    'x': 10,
    'y': 10
}
let ROVER_POINT_C = {
    'x': -10,
    'y': 10
}

function setup() {
    let canvas = createCanvas(CANVAS_WIDTH, CANVAS_HEIGHT)
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

    // canvas static
    background(225) // draw background with RGB(255,255,255)
    translate(width / 2, height / 2) // make the (0,0) point the center of canvas
    scale(zoomValue) // zoom to given value
    drawGrid() // draw the grid for reference
    circle(BASE_POSITION['x'], BASE_POSITION['y'], BASE_DIAMETER) // draw circle base station

    // canvas dynamic
    drawRover()

}

// function to draw the lines and values on canvas
// this grid can be drawn of any size, doesn't have to follow canvas values
// i.e. we can make it so that 10px = 1m and make the grid go from -1km to 1km (-10000 to 10000 pixels)
// shouldn't forget to adjust the scaling min/max so the whole grid can fit into the small canvas if needed
function drawGrid(message) {
    stroke(200)
    fill(120)
    for (let x = -width; x < width; x += 50) {
        line(x, -height, x, height)
        text(x, x + 1, 12)
    }
    for (let y = -height; y < height; y += 50) {
        line(-width, y, width, y)
        text(y, 1, y + 12)
    }
}

function drawRover() {
    checkRoverPosition() // get the heading, lat, and long values from ROS topic
    translate(roverX, roverY) // move the rover
    rotate(roverHeading) // rotate the rover
    triangle(ROVER_POINT_A['x'], ROVER_POINT_A['y'], ROVER_POINT_B['x'], ROVER_POINT_B['y'], ROVER_POINT_C['x'], ROVER_POINT_C['y']) // draw triangle rover
}

// register mouse wheel event and do the zoom
function mouseWheel(e) {
    if (winMouseX > canvasPosition['left'] && winMouseX < canvasPosition['left'] +
        width && winMouseY > canvasPosition['top'] && winMouseY < canvasPosition['top'] + height) {
        let newZoomValue = zoomValue - e.delta / 500 // delta is too big for the scale() fn, so must be divided by 500 to reduce amount fo zoom per scroll
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

    AdjustValuesToCartesian(roverLat, roverLong, roverHeading)
}

// function which converts the values into something usable by the canvas
function AdjustValuesToCartesian(roverLat, roverLong, roverHeading) {
    let direction = directionToRover(baseLat, baseLong, roverLat, roverLong)
    let distance = distanceToRover(baseLat, baseLong, roverLat, roverLong)

    roverX = PIXELS_PER_METER * distance * Math.cos(direction)
    roverY = PIXELS_PER_METER * distance * Math.sin(direction)
}
//})
