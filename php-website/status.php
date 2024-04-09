<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8" />
<script type="text/javascript" src="https://static.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js"></script>
<script type="text/javascript" src="http://static.robotwebtools.org/roslibjs/current/roslib.min.js"></script>

<script type="text/javascript">
  //console.log('I am here!');
  // Connecting to ROS
  // -----------------
  var ros = new ROSLIB.Ros();

  // Create a connection to the rosbridge WebSocket server.
  ros.connect('ws://<?php echo $_SERVER['SERVER_ADDR'] ?>:9090');

  // Publishing a Topic
  // ------------------

  // First, we create a Topic object with details of the topic's name and message type.
  var webCommands = new ROSLIB.Topic({
    ros : ros,
    name : '/webCommands',
    messageType : 'std_msgs/Float32'
  });

  

  //Subscribing to a Topic
  //----------------------

  // Like when publishing a topic, we first create a Topic object with details of the topic's name
  // and message type. Note that we can call publish or subscribe on the same topic object.
  var navMsg = new ROSLIB.Topic({
    ros : ros,
    name : '/navMsg',
    messageType : 'std_msgs/Float32'
  });

  var unlockcode = new ROSLIB.Param({
    ros : ros,
    name : '/unlock_code'
  });

/* 
  function processStatus(message) {
     
     switch (message.data) {
         case 1: //home
            document.getElementById('home').style.display = 'inline';
            document.getElementById('transit').style.display = 'none';
            document.getElementById('arrived').style.display = 'none';
            break;
         case 2: //destination
            document.getElementById('home').style.display = 'none';
            document.getElementById('transit').style.display = 'none';
            document.getElementById('arrived').style.display = 'inline';
            document.getElementByID('unlock').disabled = false;
            unlockcode.get(function(value) {
                document.getElementByID("passcode").innerHTML = value;
            });
            break;
         case 3: //in transit
            document.getElementById('home').style.display = 'none';
            document.getElementById('transit').style.display = 'inline';
            document.getElementById('arrived').style.display = 'none';
            break;
         default:
            
     }
     
  }
*/
  // Then we add a callback to be called every time a message is published on this topic.
  navMsg.subscribe(function(message) {
     console.log('Navmsg: ' + message.data);
     switch (message.data) {
         case 1: //home
            document.getElementById('home').style.display = 'inline';
            document.getElementById('transit').style.display = 'none';
            document.getElementById('arrived').style.display = 'none';
            break;
         case 2: //destination
            document.getElementById('home').style.display = 'none';
            document.getElementById('transit').style.display = 'none';
            document.getElementById('arrived').style.display = 'inline';
            document.getElementById('unlock').disabled = false;
            unlockcode.get(function(value) {
                document.getElementById("passcode").innerHTML = value;
            });
            break;
         case 3: //in transit
            document.getElementById('home').style.display = 'none';
            document.getElementById('transit').style.display = 'inline';
            document.getElementById('arrived').style.display = 'none';
            break;
         default:
            
     }
     
  });
  
  function sendUnlock(){
    var toSend = new ROSLIB.Message({data : 0});
    webCommands.publish(toSend);
    document.getElementById("unlock").disabled = true;
    //navMsg.unsubscribe();
    //ros.close();
  }

</script>

</head>

<body>
  <h1>Status</h1>
  
  <p>Delivery status:</p>
  <div id="statusIndicator">
      <p id="home" style="color:#6666ff">
      Waiting to leave
    </p>
    <p id="transit" style="color:#00D600; display:none">
      In Transit
    </p>
    <p id="arrived" style="color:#FF0000; display:none">
      Delivery has arrived
    </p>
  </div>
  
  <p>Unlock Code:</p>
  <p id="passcode">4873</p>
  
  <p> <input id="unlock" type="button" value="Remote Unlock" name="unlock_box" disabled="disabled" onclick="sendUnlock()" /> </p>
  
</body>
</html>
