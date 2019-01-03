// Console Log
const logConsole = "#write-to-log";

function appendToConsole(msg) {
    $(logConsole).append(msg + "\n");
}

function clearText() {
    $(logConsole).html("");
}

function scrollToBottom() {
    $(logConsole).scrollTop($(logConsole)[0].scrollHeight);
}

function pingRover(ping_msg, ros_msg) {
    appendToConsole(ping_msg);
    appendToConsole(ros_msg);
    scrollToBottom();
}

// Manual control
function manualControl() {
    var a = document.getElementById("ArmcontrolsOFF");
    var b = document.getElementById("ArmcontrolsON");

    if (a.style.display === "none") {
        a.style.display = "block";
        b.style.display = "none";
    } else {
        a.style.display = "none";
        b.style.display = "block";
        b.style.borderRadius = "0";
    }
}

// AJAX
function sendRequest(msg) {
    var xhr = new XMLHttpRequest();

    xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            // Typical action to be performed when the document is ready:
            console.log(JSON.parse(xhr.response));
        }
    };
    xhr.open('GET', msg, true);
    xhr.send(null);
}
