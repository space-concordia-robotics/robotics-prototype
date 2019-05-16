var canvas=document.getElementById("canvas");
var ctx=canvas.getContext("2d");
var cw=canvas.width;
var ch=canvas.height;


var PI2=Math.PI*2;
var myData = [1,2,3,4,5,6,7,8,9,10,11,12,13];
var cx=175;
var cy=175;
var radius=175;

var wheel=document.createElement('canvas');
var wheelCtx=wheel.getContext('2d');

var indicator=document.createElement('canvas');
var indicatorCtx=indicator.getContext('2d');


var angle=PI2-PI2/4; // 3pi/4

var myColor = [];
for (var i = 0; i < myData.length - 1; i++) {
    myColor.push('gold'); // randomColor()

    if (i == 11) {
        myColor.push('red')
    }
}

makeWheel();
makeIndicator();

// initialize the ting
requestAnimationFrame(animate);

function makeWheel(){

  wheel.width=wheel.height=radius*2+2;
  wheelCtx.lineWidth=1;
  wheelCtx.font='24px verdana';
  wheelCtx.textAlign='center';
  wheelCtx.textBaseline='middle';

  var cx=wheel.width/2;
  var cy=wheel.height/2;

  var sweepAngle=PI2/myData.length;
  var startAngle=0;
  for(var i=0;i<myData.length;i++){

    // calc ending angle based on starting angle
    var endAngle=startAngle+sweepAngle;

    // draw the wedge
    wheelCtx.beginPath();
    wheelCtx.moveTo(cx,cy);
    wheelCtx.arc(cx,cy,radius,startAngle,endAngle,false);
    wheelCtx.closePath();
    wheelCtx.fillStyle=myColor[i];
    wheelCtx.strokeStyle='black';
    wheelCtx.fill();
    wheelCtx.stroke();

    // draw the label
    var midAngle=startAngle+(endAngle-startAngle)/2;
    var labelRadius=radius*.85;
    var x=cx+(labelRadius)*Math.cos(midAngle);
    var y=cy+(labelRadius)*Math.sin(midAngle);
    wheelCtx.fillStyle='gold';
    wheelCtx.fillText(myData[i],x,y);
    wheelCtx.strokeText(myData[i],x,y);
    console.log("drawing", myData[i])

    // increment angle
    startAngle+=sweepAngle;
  }


}

function makeIndicator(){

  indicator.width=indicator.height=radius+radius/10;
//  indicator.width=indicator.height=radius+radius/10;
  indicatorCtx.font='18px verdana';
  indicatorCtx.textAlign='center';
  indicatorCtx.textBaseline='middle';
  indicatorCtx.fillStyle='skyblue';
  indicatorCtx.strokeStyle='blue';
  indicatorCtx.lineWidth=1;

//  var cx=indicator.width/4;
  var cx=indicator.width/2;
//  var cy=indicator.height/2;
  var cy=indicator.height/2;

  indicatorCtx.beginPath();
  indicatorCtx.moveTo(cx-radius/8,cy);
  indicatorCtx.lineTo(cx,cy+indicator.height/2); // change cy+indicator.height/2 to cy-etc. to have it point up instead of down
  indicatorCtx.lineTo(cx+radius/8,cy);
  indicatorCtx.closePath();
  indicatorCtx.fillStyle='skyblue'
  indicatorCtx.fill();
  indicatorCtx.stroke();

  indicatorCtx.beginPath();
//  indicatorCtx.arc(cx,cy,radius/3,0,PI2);
  indicatorCtx.arc(cx,cy,radius/3,0,PI2 );
  indicatorCtx.closePath();
  indicatorCtx.fill();
  indicatorCtx.stroke();

  indicatorCtx.fillStyle='blue';
  indicatorCtx.fillText('Current vial',cx,cy);
}


function animate(time){
  ctx.clearRect(0,0,cw,ch);
  ctx.translate(cw/2,ch/2);
  ctx.rotate(angle);
  ctx.drawImage(wheel,-wheel.width/2,-wheel.height/2);
  ctx.rotate(-angle);
  ctx.translate(-cw/2,-ch/2);
  ctx.drawImage(indicator,cw/2-indicator.width/2,ch/2-indicator.height/2)
  //angle+=PI2/360; // change this to -= to rotate left
  //requestAnimationFrame(animate);
}

function animateRight(time){
  ctx.clearRect(0,0,cw,ch);
  ctx.translate(cw/2,ch/2);
  ctx.rotate(angle);
  ctx.drawImage(wheel,-wheel.width/2,-wheel.height/2);
  ctx.rotate(-angle);
  ctx.translate(-cw/2,-ch/2);
  ctx.drawImage(indicator,cw/2-indicator.width/2,ch/2-indicator.height/2)
  angle+=PI2/365; // change this to -= to rotate left
  //requestAnimationFrame(animate);
}

function animateLeft(time){
  ctx.clearRect(0,0,cw,ch);
  ctx.translate(cw/2,ch/2);
  ctx.rotate(angle);
  ctx.drawImage(wheel,-wheel.width/2,-wheel.height/2);
  ctx.rotate(-angle);
  ctx.translate(-cw/2,-ch/2);
  ctx.drawImage(indicator,cw/2-indicator.width/2,ch/2-indicator.height/2)
  angle-=PI2/365; // change this to += to rotate right
  //requestAnimationFrame(animate);
}


function randomColor(){
  return('#'+Math.floor(Math.random()*16777215).toString(16));
}

function rotateRight() {
    for (var i = 0; i < 28; i++) {
        requestAnimationFrame(animateRight)
    }
}

function rotateLeft() {
    for (var i = 0; i < 28; i++) {
        requestAnimationFrame(animateLeft)
    }
}
