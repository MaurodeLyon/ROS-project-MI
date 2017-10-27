  var ros = new ROSLIB.Ros();	

  // Connecting to ROS
  // -----------------
  var ros = new ROSLIB.Ros();	
  // If there is an error on the backend, an 'error' emit will be emitted.
ros.on('error', function(error) {
    document.getElementById('connecting').style.display = 'none';
    document.getElementById('connected').style.display = 'none';
    document.getElementById('closed').style.display = 'none';
    document.getElementById('error').style.display = 'inline';
    console.log(error);
  });

  // Find out exactly when we made a connection.
  ros.on('connection', function() {
    console.log('Connection made!');
    document.getElementById('connecting').style.display = 'none';
    document.getElementById('error').style.display = 'none';
    document.getElementById('closed').style.display = 'none';
    document.getElementById('connected').style.display = 'inline';
  });

  ros.on('close', function() {
    console.log('Connection closed.');
    document.getElementById('connecting').style.display = 'none';
    document.getElementById('connected').style.display = 'none';
    document.getElementById('closed').style.display = 'inline';
  });


  // Create a connection to the rosbridge WebSocket server.
  ros.connect('ws://localhost:9090');
/*
  List of topics
*/
 
var cmdVel = new ROSLIB.Topic({
    ros : ros,
    name : '/turtle1/cmd_vel',
    messageType : 'geometry_msgs/Twist'
});

ros.getTopics(function(result) {
    
console.log(result);

    var table = document.querySelector("#topiclist");
    var tableHeader = "<tr><th>topic</th><th>data</th></tr>";

    //for (var i =0; i< result.topics.length; i++) {
    //  tableHeader += "<tr><td>" + result.topics[i] + "</td><td></td></tr>";
    //}
	
	console.log(result);
	tableHeader += "<tr>";
    table.innerHTML = tableHeader;//row;
  });

var twist = new ROSLIB.Message({
    linear : {
      x : 2.0,
      y : 0.0,
      z : 0.0
    },
    angular : {
      x : 0.0,
      y : 2.0,
      z : 0.0
    }
  });
console.log(setTopics(this.ros,'/turtle1/cmd_vel','geometry_msgs/Twist'));
function setTopics(nameTopic,name,messageType){
        nameTopic = new ROSLIB.Topic({
        ros : ros,
        name : name,
        messageType : messageType
  });
  return nameTopic;
}

function decodeMessage(msg){
	return JSON.parse(msg);
}

function parsToMessage(msg){
	return JSON.stringify(msg);
}
setTopics(this.ros,'/turtle1/cmd_vel','geometry_msgs/Twist').subscribe(function(message) {
	
	console.log(message)
});

setTopics(this.ros,'/fix','sensor_msgs/NavSatFix').subscribe(function(message) {
	var table = document.querySelector("#topiclist");
    var tableHeader = "<tr><th>topic</th><th>data</th></tr>";
	tableHeader += "<tr><td>GPS Coordinates</td><td>" + message.latitude+ "</td></tr><tr><td></td><td>"+ message.longitude+"</td></tr>";
    tableHeader += "<tr>";
    table.innerHTML = tableHeader;//row;
	//var parser= decodeMessage(message);
	var latitude = "51.586445,4.79162166667";//message.latitude +","+ message.longitude;
	document.getElementById('latlng').value = latitude;
	console.log(message.latitude);
});


function pubMessage() {
var latitude = "51.586445,4.79162166667";//message.latitude +","+ message.longitude;
	document.getElementById('latlng').value = latitude;
	console.log(message.latitude);
   var linearX = 2.0;
    var angularZ = 2.0;
    twist.linear.x = linearX;
    twist.angular.z = angularZ;

    cmdVel.publish(twist);
}

function getTopics(){
}
