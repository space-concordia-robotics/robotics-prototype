const fs = require('fs').promises
const path = require('path')
const ip = require('ip')
const nodemailer = require('nodemailer')

const RETRY_DELAY = 3000
const MAX_TRIES = 40
const EMAILS_FILE_NAME = 'emails.txt'
// fallback to default emails if there's and issue with the emails file
const DEFAULT_EMAILS = [
    'davidhuculak5@gmail.com',
    'petergranitski@gmail.com',
    'william.wells@spaceconcordia.ca'
]

let ourIP
let numTries = 1

console.log('Current time: ' + new Date().getTime())

const getEmailsFromFile = async () => {
    const filePath = path.resolve(__dirname, EMAILS_FILE_NAME)
    let emails = DEFAULT_EMAILS
    try {
        const fileString = await fs.readFile(filePath, 'utf-8')
        emails = fileString.split(',')
            .map(item => item.trim())
            .filter(item => item.length > 0)
        if (emails.length < 1) {
            console.log(`Warning: no emails found in ${filePath}. Falling back to default emails:`)
            console.log(DEFAULT_EMAILS)
            emails = DEFAULT_EMAILS
        }
    } catch (e) {
        if (e.code === 'ENOENT') {
            console.log(`Warning: ${filePath} not found. Falling back to default emails:`)
            console.log(DEFAULT_EMAILS)
        }
    }
    return emails
}

const tryToGetIP = () => {

    ourIP = ip.address()

    if (ourIP === '127.0.0.1') {
        console.log('error getting IP address')
        if (numTries < MAX_TRIES) {
            console.log('Trying again in 3 seconds... (MAX_TRIES = ' + MAX_TRIES + ')\n')
            setTimeout(function () {
                numTries++
                tryToGetIP()
            }, RETRY_DELAY)
        } else {
            console.log('Too many unsuccessful attempts at getting the IP address. Closing program')
            process.exit(1)
        }
    } else {
        numTries = 1
        console.log('Succeeded in getting IP address (' + ourIP + '). Sending it via email.')
        sendEmail(ourIP)
    }
}

const sendEmail = async ip => {

    let message = 'The IP address is: ' + ip

    let transporter = nodemailer.createTransport({
        host: 'smtp.gmail.com',
        port: 465,
        secure: true, // secure:true for port 465, secure:false for port 587
        auth: {
            user: 'concordiacourseplanner@gmail.com',
            pass: 'tranzone'
        }
    })

    const emails = await getEmailsFromFile()
    let mailOptions = {
        from: '"Mr. odroid sir" <concordiacourseplanner@gmail.com>', // sender address
        to: emails.join(', '), // list of receivers
        subject: 'The IP Address of the odroid', // Subject line
        html: message // html body
    }

    transporter.sendMail(mailOptions, function (error, info) {
        if (error) {
            if (numTries < MAX_TRIES) {
                numTries++
                console.log('Failed to send email. Trying again in 1 second')
                setTimeout(function () {
                    console.log('Trying to send email again')
                    sendEmail(ip)
                }, 1000)
            } else {
                return console.log(error)
            }
        } else {
            console.log('Message %s sent: %s', info.messageId, info.response)
            console.log('Current time: ' + new Date().getTime())
        }
    })
}

tryToGetIP()
