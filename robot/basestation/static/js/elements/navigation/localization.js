//$(document).ready(() => {
let rover_heading = 0
let rover_x = 0
let rover_y = 0
let zoom_value = 1
let MAX_ZOOM = 2
let MIN_ZOOM = 0.25
let canvas_pos
let keys = { //javascript keycodes
    'r': 82,
    'w': 87,
    'a': 65,
    's': 83,
    'd': 68
}
let PIXELS_PER_METER = 5
let base_lat = $('#antenna-latitude').text()
let base_long = $('#antenna-longitude').text()
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
    canvas_pos = $('#defaultCanvas0').offset()
    angleMode(DEGREES) // makes p5.js use DEG instead of RAD
}

function draw() {
    // all of the manual controls
    if (keyIsDown(keys['r'])) { //reset position
        rover_heading = 0;
        zoom_value = 1;
        rover_x = rover_y = 0;
    }
    if (keyIsDown(keys['a'])) { // spin ccw
        rover_heading -= 1
    }
    if (keyIsDown(keys['d'])) { // spin cw
        rover_heading += 1
    }
    if (keyIsDown(keys['w'])) { // move forward
        rover_x += sin(rover_heading);
        rover_y -= cos(rover_heading);
    }
    if (keyIsDown(keys['s'])) { // move backward
        rover_x -= sin(rover_heading);
        rover_y += cos(rover_heading);
    }

    // canvas static
    background(225) // draw background with RGB(255,255,255)
    translate(width / 2, height / 2) // make the (0,0) point the center of canvas
    scale(zoom_value) // zoom to given value
    draw_grid() // draw the grid for reference
    circle(BASE_POSITION['x'], BASE_POSITION['y'], BASE_DIAMETER) // draw circle base station

    // canvas dynamic
    draw_rover()

}

// function to draw the lines and values on canvas
// this grid can be drawn of any size, doesn't have to follow canvas values
// i.e. we can make it so that 10px = 1m and make the grid go from -1km to 1km (-10000 to 10000 pixels)
// shouldn't forget to adjust the scaling min/max so the whole grid can fit into the small canvas if needed
function draw_grid(message) {
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

function draw_rover() {
    check_rover_position() // get the heading, lat, and long values from ROS topic
    translate(rover_x, rover_y) // move the rover
    rotate(rover_heading) // rotate the rover
    triangle(ROVER_POINT_A['x'], ROVER_POINT_A['y'], ROVER_POINT_B['x'], ROVER_POINT_B['y'], ROVER_POINT_C['x'], ROVER_POINT_C['y']) // draw triangle rover
}

// register mouse wheel event and do the zoom
function mouseWheel(e) {
    if (winMouseX > canvas_pos['left'] && winMouseX < canvas_pos['left'] +
        width && winMouseY > canvas_pos['top'] && winMouseY < canvas_pos['top'] + height) {
        let newzoom_value = zoom_value - e.delta / 500 // delta is too big for the scale() fn, so must be divided by 500 to reduce amount fo zoom per scroll
        if (newzoom_value >= MIN_ZOOM && newzoom_value <= MAX_ZOOM) {
            zoom_value = newzoom_value
            return false
        }
    }
}

// function that gets the data from the rover_position topic
function check_rover_position() {
    let rover_lat = $('#rover-latitude').text()
    let rover_long = $('#rover-longitude').text()
    rover_heading = $('#rover-heading').text()

    adjust_values_to_cartesian(rover_lat, rover_long, rover_heading)
}

// function which converts the values into something usable by the canvas
function adjust_values_to_cartesian(rover_lat, rover_long, rover_heading) {
    let direction = direction_to_rover(base_lat, base_long, rover_lat, rover_long)
    let distance = distance_to_rover(base_lat, base_long, rover_lat, rover_long)

    rover_x = PIXELS_PER_METER * distance * Math.cos(direction)
    rover_y = PIXELS_PER_METER * distance * Math.sin(direction)
}
//})
