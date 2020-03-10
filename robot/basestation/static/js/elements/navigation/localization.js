let rotationValue = 0;
let zoomValue = 1;
let x = 0
let y = 0;
let headingElt;

function setup(){
    let canvas = createCanvas(450,300);
    canvas.parent('#canvasContainer');
    angleMode(DEGREES);
}


function draw() {
    // keycodes
    // q = 81, e = 69, r = 82, w = 87, a = 65, s = 83, d = 68
    if (keyIsDown(82)) {rotationValue = 0; zoomValue = 1; x = y = 0;} //reset position
    if (keyIsDown(65)) {rotationValue += 1} // spin ccw
    if (keyIsDown(68)) {rotationValue -= 1} // spin cw
    adjustRotationValue(); // keep rotationValue within 0-360
    if (keyIsDown(87)) {x+=sin(rotationValue) ; y+=cos(rotationValue);} // move forward
    if (keyIsDown(83)) {x-=sin(rotationValue) ; y-=cos(rotationValue);} // move backward
    
    background(225); // draw background
    translate(width/2, height/2) // make the (0,0) the center point
    scale(zoomValue); // scale to given value
    
    triangle(0,-15,10,10,-10,10); // draw triangle
    
    rotate(rotationValue); // rotate everything but triangle
    translate(x, y); // move everything but triangle
    
    drawGrid(); // draw the grid for reference
    //headingElt.html("Heading: " + -rotationValue + '&#176;'); // Set the heading value in the HTML
}

// function to draw the line and values on canvas
function drawGrid() {
    push();
	stroke(200);
	fill(120);
	for (var x=-width; x < width; x+=50) {
		line(x, -height, x, height);
		text(x, x+1, 12);
	}
	for (var y=-height; y < height; y+=50) {
		line(-width, y, width, y);
		text(y, 1, y+12);
	}
    pop();
}

// register mouse wheel event and do the zoom
function mouseWheel(e) {
    let newZoomValue = zoomValue + e.delta/500;
    if (newZoomValue >= 0.5 && newZoomValue <= 2) {
        zoomValue = newZoomValue;
        return false;
    }
}
        
// function which keeps heading within 0-360
function adjustRotationValue() {
    if (-rotationValue < 0) {
        rotationValue -= 360;
    }
    if (-rotationValue == 360) {
        rotationValue += 360;
    } 
}
        
        
        
        
        
        
        
        
        
        