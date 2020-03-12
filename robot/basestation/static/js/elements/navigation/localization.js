let map_rotation_deg = 0;
let zoom_value = 1;
let rover_x = 0
let rover_y = 0;
let canvas_pos;
let keys = {'r': 82, 'w': 87, 'a': 65, 's': 83, 'd': 68} //javascript keycodes


function setup(){
    let canvas = createCanvas(450,300);
    canvas.parent('#canvasContainer');
    canvas_pos = $('#defaultCanvas0').offset();
    angleMode(DEGREES);
}


function draw() {
    if (keyIsDown(keys['r'])) {map_rotation_deg = 0; zoom_value = 1; rover_x = rover_y = 0;} //reset position
    
    if (keyIsDown(keys['a'])) {map_rotation_deg += 1} // spin ccw
    if (keyIsDown(keys['d'])) {map_rotation_deg -= 1} // spin cw
    adjustmap_rotation_deg(); // keep map_rotation_deg within 0-360
    
    if (keyIsDown(keys['w'])) {rover_x += sin(map_rotation_deg); rover_y += cos(map_rotation_deg);} // move forward
    if (keyIsDown(keys['s'])) {rover_x -= sin(map_rotation_deg); rover_y -= cos(map_rotation_deg);} // move backward
    
    background(225); // draw background
    translate(width/2, height/2) // make the (0,0) point the center of canvas
    scale(zoom_value); // zoom to given value
    
    triangle(0,-15,10,10,-10,10); // draw triangle rover
    
    rotate(map_rotation_deg); // rotate everything relative to rover
    translate(rover_x, rover_y); // move everything relative to rover
    
    drawGrid(); // draw the grid for reference
    circle(0,0,10); // draw circle base station
}

// function to draw the lines and values on canvas
function drawGrid(message) {
	stroke(200);
	fill(120);
	for (let x=-width; x < width; x+=50) {
		line(x, -height, x, height);
		text(x, x+1, 12);
	}
	for (let y=-height; y < height; y+=50) {
		line(-width, y, width, y);
		text(y, 1, y+12);
	}
}


// register mouse wheel event and do the zoom
function mouseWheel(e) {
    if (winMouseX > canvas_pos['left'] && winMouseX < canvas_pos['left'] + width && winMouseY > canvas_pos['top'] && winMouseY < canvas_pos['top'] + height) {
        let newzoom_value = zoom_value - e.delta/500;
        if (newzoom_value >= 0.25 && newzoom_value <= 4) {
            zoom_value = newzoom_value;
            return false;
        }
    }
}
        
// function which keeps heading within 0-360
function adjustmap_rotation_deg() {
    if (-map_rotation_deg < 0) {
        map_rotation_deg -= 360;
    }
    if (-map_rotation_deg == 360) {
        map_rotation_deg += 360;
    } 
}   