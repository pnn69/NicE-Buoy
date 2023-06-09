//var gateway = `ws://${window.location.hostname}/ws`;
//var gateway = `ws://195.114.237.138/ws`;
//var gateway = `ws://192.168.178.132/ws`;
var gateway = `ws://192.168.1.24/ws`;
var websocket;
window.addEventListener("load", onLoad);

function getValues() {
    websocket.send(JSON.stringify({
        GETSTATUS: 1
    }));
}

function initWebSocket() {
    console.log("Trying to open a WebSocket connection...");
    websocket = new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage; // <-- add this line
}

function onOpen(event) {
    console.log("Connected");
    toStatus("Connected");
    getValues();
}

function onClose(event) {
    console.log("Connection closed");
    toStatus("No Connection");
    setTimeout(initWebSocket, 2000);
}

function onMessage(event) {
    //console.log(event);
    console.log(event.data);
    //toStatus(event.data);
    var stuff = JSON.parse(event.data);
    var val = stuff["STATUS1"];
    if (val != undefined) {
        if (val == 2) {
            radiobtn = document.getElementById("Lock_Buoy_1");
            radiobtn.checked = true;
        } else if (val == 6) {
            radiobtn = document.getElementById("Remote_Buoy_1");
            radiobtn.checked = true;
            document.getElementById("Trottle1").value = 0;
            document.getElementById("trottle1").innerHTML = 0;

        } else if (val == 7) {
            radiobtn = document.getElementById("Doc_Buoy_1");
            radiobtn.checked = true;
        } else {
            radiobtn = document.getElementById("Idle_1");
            radiobtn.checked = true;
        }
    }

    var val = stuff["speed1"];
    if (val != undefined) {
        document.getElementById("Trottle1").value = val;
        document.getElementById("trottle1").innerHTML = val;
    }
    var val = stuff["ddir1"];
    if (val != undefined) {
        document.getElementById("Rudder1").value = val;
        document.getElementById("rudder1").innerHTML = val;
    }
    var val = stuff["tgdir1"];
    if (val != undefined) {
        document.getElementById("tdir1").innerHTML = val;
    }
    var val = stuff["mdir1"];
    if (val != undefined) {
        document.getElementById("mdir1").innerHTML = val;
    }
    var val = stuff["rssi1"];
    if (val != undefined) {
        document.getElementById("rssi1").innerHTML = val;
    }
    var val = stuff["tgdistance1"];
    if (val != undefined) {
        document.getElementById("tgdistance1").innerHTML = val;
    }
    var val = stuff["speedbb1"];
    if (val != undefined) {
        if (val > 0) {
            bar.style.backgroundColor = 'green';
            console.log("green");
        } else {
            bar.style.backgroundColor = 'red';
            console.log("red");
            val = val * -1;
        }
        document.getElementById("bb1").innerHTML = val + "%";
        document.getElementById("bb1").style = "height:" + val + "%";
    }

    var val = stuff["speedsb1"];
    if (val != undefined) {
        if (val > 0) {
            bar.style.backgroundColor = 'green';
            console.log("green");
        } else {
            bar.style.backgroundColor = 'red';
            console.log("red");
            val = val * -1;
        }
        document.getElementById("sb1").innerHTML = val + "%";
        document.getElementById("sb1").style = "height:" + val + "%";
    }

}

function onLoad(event) {
    initWebSocket();
    toStatus("No connection");
}

document.getElementById("Lock_Buoy_1").onclick = function () {
    var x = document.getElementById("LOOK_Buoy_1");
    websocket.send(JSON.stringify({
        LOCKEDBUOY1: true
    }));
    websocket.send(JSON.stringify({
        Lock_Buoy_1: true
    }));
}
document.getElementById("Remote_Buoy_1").onclick = function () {
    websocket.send(JSON.stringify({
        REMOTEBUOY1: true
    }));
    websocket.send(JSON.stringify({
        Remote_Buoy_1: true
    }));
}
document.getElementById("Doc_Buoy_1").onclick = function () {
    websocket.send(JSON.stringify({
        DOCBUOY1: true
    }));
    websocket.send(JSON.stringify({
        Doc_Buoy_1: true
    }));
}
document.getElementById("Idle_1").onclick = function () {
    websocket.send(JSON.stringify({
        IDLEBUOY1: true
    }));
    websocket.send(JSON.stringify({
        Idle_1: true
    }));
}


document.getElementById("Rudder1").onchange = function () {
    var x = document.getElementById("Rudder1");
    websocket.send(JSON.stringify({
        Rudder1: x.value
    }));
}

document.getElementById("Trottle1").onchange = function () {
    var x = document.getElementById("Trottle1");
    websocket.send(JSON.stringify({
        Speed1: x.value
    }));
}

function toStatus(txt) {
    document.getElementById("status").innerHTML = "Status:" + txt;
}