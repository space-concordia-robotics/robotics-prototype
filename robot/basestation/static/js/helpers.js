function entertext() {
    document.getElementById('write-to-log').value += "Test log ...\n";
}

function cleartext() {
    document.getElementById('write-to-log').value = "";
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
