const logConsole = "#write-to-log";

function appendToConsole(msg) {
    $(logConsole).append(msg + "\n");
}

function cleartext() {
    $(logConsole).html("");
}

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

function sendRequest(msg) {
    var xhr = new XMLHttpRequest();

    xhr.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            // Typical action to be performed when the document is ready:
            console.log(xhr.responseText);
        }
    };
    xhr.open('GET', msg, true);
    xhr.send(null);
}
