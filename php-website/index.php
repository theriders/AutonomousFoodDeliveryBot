<!DOCTYPE html>

<html>
    <head>
        <title>AutoVendors Intro</title>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <script type="text/javascript" src="https://static.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js"></script>
        <script type="text/javascript" src="http://static.robotwebtools.org/roslibjs/current/roslib.min.js"></script>

        <script type="text/javascript">
            console.log('I am here!');
            // Connecting to ROS
            // -----------------
            var ros = new ROSLIB.Ros();

            // If there is an error on the backend, an 'error' emit will be emitted.
            ros.on('error', function (error) {
                document.getElementById('connecting').style.display = 'none';
                document.getElementById('connected').style.display = 'none';
                document.getElementById('error').style.display = 'inline';
                console.log(error);
            });

            // Find out exactly when we made a connection.
            ros.on('connection', function () {
                console.log('Connection made!');
                document.getElementById('connecting').style.display = 'none';
                document.getElementById('error').style.display = 'none';
                document.getElementById('connected').style.display = 'inline';
                document.getElementById('continue').disabled = false;
            });

            ros.on('close', function () {
                console.log('Connection closed.');
                document.getElementById('connecting').style.display = 'none';
                document.getElementById('connected').style.display = 'none';
            });

            // Create a connection to the rosbridge WebSocket server.
            ros.connect('ws://<?php echo $_SERVER['SERVER_ADDR'] ?>:9090');

	    function redirectPage() {
                ros.close();
		setTimeout('window.location.replace("order_form.php");',2000);
            }
        </script>

    </head>
    <body>
        <h1>AutoVendors Order System</h1> 
        <p>This is the ordering website for the Autonomous Vending Machine Delivery Robot. Please click "Place an Order".</p>
        <div id="statusIndicator">
            <p id="connecting">
                Connecting to delivery bot...
            </p>
            <p id="connected" style="color:#00D600; display:none">
                Connected
            </p>
            <p id="error" style="color:#FF0000; display:none">
                Please refresh the page.
            </p>
        </div>
        <p> <input id="continue" type="submit" value="Place an Order" name="continue" disabled="disabled" onclick="redirectPage()" /> </p>
    </body>
</html>
