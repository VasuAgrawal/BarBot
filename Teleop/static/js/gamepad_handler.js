var ws;
var doc_elem = document.getElementById("joystick_state");

function start_ws() {
    ws = new ReconnectingWebSocket("ws://" + location.host + "/gamepad",
        null, {});

    ws.onopen = function() {
        console.log("Successfully connected to the server at " + location.host);
        requestAnimationFrame(gamepadHandlerConstant);
    }

    ws.onclose = function() {
        console.log("Closed connection to the server at " + location.host);
        cancelRequestAnimationFrame(gamepadHandlerConstant);
    }

    ws.onmessage = function(e) {
        console.log("Received message " + e);
    }

    heartbeat_ws = new ReconnectingWebSocket("ws://" + location.host + 
                "/heartbeat", null, {});

    heartbeat_ws.onopen = function() {
        setInterval(function() {
            console.log("Sending heartbeat message");
            heartbeat_ws.send("Don't kill me pl0x");
        }, 500);
    }
}

function gamepadHandlerConstant() {
    for (var i = 0, len = navigator.getGamepads().length; i < len; i++) {
        var gamepad = navigator.getGamepads()[i];

        if (gamepad && gamepad.connected) {

            var state = {
                "index" : gamepad.index,
                "timestamp" : gamepad.timestamp,
                "axes" : gamepad.axes.map(function(elem) {
                    return Math.round(elem * 1000) / 1000;
                }),
                "buttons" : gamepad.buttons.map(function(elem) {
                    return elem.pressed ? 1.0 : 0.0;
                }),
                
            }

            var text = JSON.stringify(state);
            doc_elem.innerHTML = text;
            ws.send(JSON.stringify(state));
        }
    }

    requestAnimationFrame(gamepadHandler);
}

function gamepadHandler() {
    if (!("timestamps" in this)) {
        this.timestamps = [];
    }

    for (var i = 0, len = navigator.getGamepads().length; i < len; i++) {

        var gamepad = navigator.getGamepads()[i];
        if (!(this.timestamps[i])) {
            this.timestamps[i] = 0;
        }

        // Timestamp is in milliseconds. We impose a false 200 ms latency to
        // reduce congestion on the line.
        if (gamepad && gamepad.connected && 
            gamepad.timestamp > this.timestamps[i]) {
            
            this.timestamps[i] = gamepad.timestamp;

            var state = {
                "index" : gamepad.index,
                "timestamp" : gamepad.timestamp,
                "axes" : gamepad.axes.map(function(elem) {
                    return Math.round(elem * 1000) / 1000;
                }),
                "buttons" : gamepad.buttons.map(function(elem) {
                    return elem.pressed ? 1.0 : 0.0;
                }),
            }

            var text = JSON.stringify(state);
            doc_elem.innerHTML = text;
            //console.log(JSON.stringify(state));
            ws.send(JSON.stringify(state));
        }
    }

    requestAnimationFrame(gamepadHandler);
}

start_ws();
