


var ip = require('ip');
var nodemailer = require('nodemailer');

var RETRY_DELAY = 3000;
var MAX_TRIES = 40;

var ourIP;
var numTries = 1;

console.log('Current time: ' + new Date().getTime());

tryToGetIP();

function tryToGetIP(){

    ourIP = ip.address();

    if(ourIP === '127.0.0.1') {
        console.log('error getting IP address');
        if(numTries < MAX_TRIES){
            console.log('Trying again in 3 seconds... (MAX_TRIES = ' + MAX_TRIES + ')\n');
            setTimeout(function() {
                numTries++;
                tryToGetIP();
            }, RETRY_DELAY);
        } else {
            console.log('Too many unsuccessful attempts at getting the IP address. Closing program');
            process.exit(1);
        }
    } else {
        numTries = 1;
        console.log('Succeeded in getting IP address (' + ourIP + '). Sending it via email.');
        sendEmail(ourIP);
    }
}

function sendEmail(ip){

    var message = "The IP address is: " + ip;

    var transporter = nodemailer.createTransport({
        host: 'smtp.gmail.com',
        port: 465,
        secure: true, // secure:true for port 465, secure:false for port 587
        auth: {
            user: 'concordiacourseplanner@gmail.com',
            pass: 'tranzone'
        }
    });

    var mailOptions = {
        from: '"Mr. odroid sir" <concordiacourseplanner@gmail.com>', // sender address
        to: 'davidhuculak5@gmail.com, petergranitski@gmail.com, samuel.beaubien@hotmail.com', // list of receivers
        subject: 'The IP Address of the odroid', // Subject line
        html: message // html body
    };

    transporter.sendMail(mailOptions, function(error, info){
        if (error) {
            if(numTries < MAX_TRIES){
                numTries++;
                console.log('Failed to send email. Trying again in 1 second');
                setTimeout(function() {
                    console.log('Trying to send email again');
                    sendEmail(ip);
                }, 1000);
            } else {
                return console.log(error);
            }
        } else {
            console.log('Message %s sent: %s', info.messageId, info.response);
            console.log('Current time: ' + new Date().getTime());
        } 
    });
}
