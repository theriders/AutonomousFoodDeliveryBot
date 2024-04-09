<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8" />
<script type="text/javascript" src="https://static.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js"></script>
<script type="text/javascript" src="http://static.robotwebtools.org/roslibjs/current/roslib.min.js"></script>

<script type="text/javascript">
  // Connecting to ROS
  // -----------------
  
  var ros = new ROSLIB.Ros();

  //Create a connection to the rosbridge WebSocket server.
  ros.connect('ws://<?php echo $_SERVER['SERVER_ADDR'] ?>:9090');

  // Publishing a Topic
  // ------------------

  // First, we create a Topic object with details of the topic's name and message type.
  var webOrders = new ROSLIB.Topic({
    ros : ros,
    name : '/webOrders',
    messageType : 'std_msgs/String'
  });

  function publishOrder() {
      
      document.getElementById('pubButton').disabled = true;
      var x = document.getElementById("locations").selectedIndex;
	  var toSend = new ROSLIB.Message({
		  data : document.getElementsByTagName("option")[x].value});
      //alert(document.getElementsByTagName("option")[x].value);
      webOrders.publish(toSend);
  //    setTimeout('ros.close();',3000);
      setTimeout('window.location.href="status.php";',4000);
  }

</script>
</head>

<body>
  <h1>Order</h1>
  <p>Please choose your location</p>
  
  Select the delivery location:
  <select id="locations">
      <option value="C456">C456</option>
      <option value="C444D">C444-D</option>
      <option value="W417">W417</option>
      <option value="E474">E474</option>
      <option value="E463A">E463A</option>
  </select>
  
  <p>The order can not be cancelled once placed. Please check your selections.</p>
      
  <button type="button" id="pubButton" onclick="publishOrder()">Place Order</button>
  
</body>
</html>
