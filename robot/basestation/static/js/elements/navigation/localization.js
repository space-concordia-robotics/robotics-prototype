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

let base_lat
let base_long
$(document).ready(() => {
    // the elements might not be initialized yet, must test
    base_lat = $('#antenna-latitude').text()
    base_long = $('#antenna-longitude').text()
})

function setup() {
    let canvas = createCanvas(450, 300)
    canvas.parent('#canvasContainer')
    canvas_pos = $('#defaultCanvas0').offset()
    angleMode(DEGREES)
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
    adjustmap_rotation_deg() // keep rover_heading within 0-360
    if (keyIsDown(keys['w'])) { // move forward
        rover_x += sin(rover_heading);
        rover_y -= cos(rover_heading);
    }
    if (keyIsDown(keys['s'])) { // move backward
        rover_x -= sin(rover_heading);
        rover_y += cos(rover_heading);
    }

    // canvas static
    background(225) // draw background
    translate(width / 2, height / 2) // make the (0,0) point the center of canvas
    scale(zoom_value) // zoom to given value
    draw_grid() // draw the grid for reference
    circle(0, 0, 10) // draw circle base station

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
    translate(rover_x, rover_y) // move to rover position
    rotate(rover_heading) // rotate to rover heading
    triangle(0, -15, 10, 10, -10, 10) // draw triangle rover
}

// register mouse wheel event and do the zoom
function mouseWheel(e) {
    if (winMouseX > canvas_pos['left'] && winMouseX < canvas_pos['left'] + width && winMouseY > canvas_pos['top'] && winMouseY < canvas_pos['top'] + height) {
        let newzoom_value = zoom_value - e.delta / 500
        if (newzoom_value >= MIN_ZOOM && newzoom_value <= MAX_ZOOM) {
            zoom_value = newzoom_value
            return false
        }
    }
}

// function which keeps heading within 0-360
function adjustmap_rotation_deg() {
    if (-rover_heading < 0) {
        rover_heading -= 360
    }
    if (-rover_heading == 360) {
        rover_heading += 360
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
    let angle = Direction(base_lat, base_long, rover_lat, rover_long)
    let distance = Distance(base_lat, base_long, rover_lat, rover_long)

    // multiply result by 5 to move 5 pixels per meter
    // needs to be confirmed
    rover_x = 5 * distance * Math.cos(angle)
    rover_y = 5 * distance * Math.sin(angle)
}
