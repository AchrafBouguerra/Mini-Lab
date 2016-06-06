////27/08/2015 21.21/////
(function(ext) {
   $.getScript('http://cdn.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js');
   $.getScript('http://cdn.robotwebtools.org/roslibjs/current/roslib.min.js');
   //$.getScript('http://cdn.robotwebtools.org/EaselJS/current/easeljs.min.js');
   //$.getScript('http://cdn.robotwebtools.org/ros2djs/current/ros2d.min.js');

    //Ros Connection Vars
    var ros;
    var TestRosConnection=false;
    //Minilab Position Vars
    var positionX=0;
    var positionY=0;
    var positionZ=0;
    var rotationX=0;
    var rotationY=0;
    var rotationZ=0;
    var rotationW=0;
    //video streaming settings Vars
    var streaming_width=400;
    var streaming_height=200;
    var streaming_quality=90;
    //Map streaming settings Vars
    var map_width=430;
    var map_height=280;
    var map_refresh=1;
    //Laser values Vars
    var laser_min_angle=0;
    var laser_max_angle=0;
    var laser_inc_angle=0;
    var laser_time_inc=0;
    var laser_scan_time=0;
    var laser_max_range=0;
    var laser_min_range=0;
    var laser_angle_range=0;
    
    // Cleanup function when the extension is unloaded
    ext._shutdown = function() {};
    
    // Status reporting
 code
    // Use this to report missing hardware, plugin or unsupported browser
    ext._getStatus = function() {return {status: 2, msg: 'Ready'};};
    
    //Starting connection to websocket server
    //must first launch: "roslaunch rosbridge_server rosbridge_websocket.launch"
    //connecting to ROS.
    ext.RosConnection = function(adress,port){
        try {
        ros = new ROSLIB.Ros({
        url : 'ws://'+adress+':'+port 
        });
        
        console.log('Loading '+'ws://'+adress+':'+port);
        } catch (err) {console.log('Unable to connecte to websocket')};
        
        ros.on('connection', function() {
        console.log('Connected to websocket server.');
        TestRosConnection=true;  
        });

        ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
        TestRosConnection=false;
        });

        ros.on('close', function() {
        console.log('Connection to websocket server closed.');
        TestRosConnection=false;
        });
    };


//First we check to see if the browser supports the Web Speech API by checking if the webkitSpeechRecognition object exists. If not, we suggest the user upgrades his browser. 


if (!('webkitSpeechRecognition' in window)) {
  upgrade();
} else {
  var recognition = new webkitSpeechRecognition();
  recognition.continuous = true;
  recognition.interimResults = true;

  recognition.onstart = function() { ... }
  recognition.onresult = function(event) { ... }
  recognition.onerror = function(event) { ... }
  recognition.onend = function() { ... }
  


   //Test websocket connection
    ext.TestConnection = function() {
        try{
        ros.on('connection', function() {
        console.log('Connected to websocket server.');
        TestRosConnection=true;  
        });

        ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
        TestRosConnection=false;
        });

        ros.on('close', function() {
        console.log('Connection to websocket server closed.');
        TestRosConnection=false;
        });
   
        }catch(err){}
       return TestRosConnection;
       
    };
// by clicking the microphon logo to start the speech the user will trigger this code
       function startButton(event) {
       
      final_transcript = '';
      recognition.lang = select_dialect.value;
       recognition.start();

//at this point we active the voice rcognizer by recognition.start()
  recognition.onresult = function(event) {
    var interim_transcript = '';

    for (var i = event.resultIndex; i < event.results.length; ++i) {
      if (event.results[i].isFinal) {
        final_transcript += event.results[i][0].transcript;
      } else {
        interim_transcript += event.results[i][0].transcript;
      }
    }
    final_transcript = capitalize(final_transcript);
    final_span.innerHTML = linebreak(final_transcript);
    interim_span.innerHTML = linebreak(interim_transcript);
  };
 }
// the condition is fullfilled
if (interim_transcript == 'move'){

//Moving the robot: publishing to cmd_vel
    ext.MoveRobot = function(direction,speed) {
       try{
        var cmdVel = new ROSLIB.Topic({
        ros : ros,
        name : '/cmd_vel',
        messageType : 'geometry_msgs/Twist'
        });
        var twist = new ROSLIB.Message({
        linear : {x : 0,y : 0,z : 0},
        angular : {x : 0,y : 0,z : 0}
        });
        
if (direction == "Forward"){
   twist.linear.x=speed;
   console.log('Forward');
   
} 
else if (direction == "Backward"){
    twist.linear.x=-speed;
    console.log('Backward');
}  
else if (direction == "Right"){
    twist.angular.z=-speed;
    console.log('Right');
}  
else if (direction == "Left"){
    twist.angular.z=speed;
    console.log('Left');
}  
cmdVel.publish(twist);
console.log("Publishing cmd_vel");
    }catch(err) {console.log("Unable to Run MoveRobot Block")}; 
};}else console.log('say that once agian please');



  // Block and block menu descriptions
    var descriptor = {
        blocks: [
            // Block type, block name, function name
            ["h", "When connected to Mini-Lab","TestConnection"],
            ["", "Connect to Mini-Lab Adress: %s Port: %n","RosConnection","localhost","9090"],
            ["", "Move Mini-Lab Direction %m.direction_menu Speed %n", "MoveRobot","Forward",0.2],
 ],
        menus: {
            "direction_menu":["Forward","Backward","Right","Left"]
        },
    };

    // Register the extension
    ScratchExtensions.register('Enova Mini-Lab', descriptor, ext);
})({});







    
