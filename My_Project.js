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
//

<script src="//cdnjs.cloudflare.com/ajax/libs/annyang/2.4.0/annyang.min.js"></script>
<script>
if (annyang) {
  // Let's define our first command. First the text we expect, and then the function it should call
  var commands = {
    'show tps report': function() {
      $('#tpsreport').animate({bottom: '-100px'});
    }
  };

  // Add our commands to annyang
  annyang.addCommands(commands);

  // Start listening. You can call this here, or attach this call to an event, button, etc.
  annyang.start();
}
</script>
//
    
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
};
//Stopping the robot: publishing 0 to cmd_vel
ext.StopRobot = function(direction,speed) {
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
        cmdVel.publish(twist);
        console.log("Publishing cmd_vel");
        }catch(err) {console.log("Unable to Run StopRobot Block")}; 
};

//Getting the Mini-Lab's Position from topic /odom/pose/pose/position
ext.GetMinilabPositionX = function() {
try{
   var pose_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/odom',
    messageType : 'nav_msgs/Odometry'
  });
  pose_listener.subscribe(function(message) {
    console.log('/odom/pose/pose/position:'+message.pose.pose.position.x);
    positionX=message.pose.pose.position.x;
    pose_listener.unsubscribe();
  });
  return positionX;
 }catch(err){console.log("Unable to Get Position.X")};
};
ext.GetMinilabPositionY = function() {
try{
   var pose_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/odom',
    messageType : 'nav_msgs/Odometry'
  });
  
  pose_listener.subscribe(function(message) {
    console.log('/odom/pose/pose/position.y:'+message.pose.pose.position.y);
    positionY=message.pose.pose.position.y;
    pose_listener.unsubscribe();
  });
  return positionY;
 }catch(err){console.log("Unable to Get Position.Y")};
};
ext.GetMinilabPositionZ = function() {
try{
   var pose_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/odom',
    messageType : 'nav_msgs/Odometry'
  });
  
  pose_listener.subscribe(function(message) {
    console.log('/odom/pose/pose/position:'+message.pose.pose.position.z);
    positionZ=message.pose.pose.position.z;
    pose_listener.unsubscribe();
  });
  return positionZ;
 }catch(err){console.log("Unable to Get Position.Z")};
};

//Getting the Mini-Lab's Orientation from topic /odom/pose/pose/orientation
ext.GetMinilabRotationX = function() {
try{
   var pose_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/odom',
    messageType : 'nav_msgs/Odometry'
  });
  
  pose_listener.subscribe(function(message) {
    console.log('/odom/pose/pose/orientation:'+message.pose.pose.orientation.x);
    rotationX=message.pose.pose.orientation.x;
    pose_listener.unsubscribe();
  });
  return rotationX;
 }catch(err){console.log("Unable to Get Rotation.X")};
};

ext.GetMinilabRotationY = function() {
try{
   var pose_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/odom',
    messageType : 'nav_msgs/Odometry'
  });
  
  pose_listener.subscribe(function(message) {
    console.log('/odom/pose/pose/orientation:'+message.pose.pose.orientation.y);
    rotationY=message.pose.pose.orientation.y;
    pose_listener.unsubscribe();
  });
  return rotationY;
 }catch(err){console.log("Unable to Get rotation.Y")};
};

ext.GetMinilabRotationZ = function() {
try{
   var pose_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/odom',
    messageType : 'nav_msgs/Odometry'
  });
  
  pose_listener.subscribe(function(message) {
    console.log('/odom/pose/pose/orientation:'+message.pose.pose.orientation.z);
    rotationZ=message.pose.pose.orientation.z;
    pose_listener.unsubscribe();
  });
  return rotationZ;
 }catch(err){console.log("Unable to Get rotation.Z")};
};

ext.GetMinilabRotationW = function() {
try{
   var pose_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/odom',
    messageType : 'nav_msgs/Odometry'
  });
  
  pose_listener.subscribe(function(message) {
    console.log('/odom/pose/pose/orientation:'+message.pose.pose.orientation.w);
    rotationW=message.pose.pose.orientation.w;
    pose_listener.unsubscribe();
  });
  return rotationW;
 }catch(err){console.log("Unable to Get rotation.W")};
};

//stream video using web_video_server
//must run first: "rosrun web_video_server web_video_server" 
ext.StreamVideo = function(adress,port) {
   try{
       console.log("start Streaming")
       window.open('http://'+adress+':'+port+'/stream_viewer?topic=/camera/rgb/image_raw&width='+streaming_width+'&height='+streaming_height+'&quality='+streaming_quality,'Streaming video From Mini-Lab',"resizable=yes,toolbar=no,,titlebar=no,location=no,menubar=no,scrollbars=no,top=20,left=20,width=450,height=300");
       }catch(err){console.log("Unable to Run Streaming Block")};
      };
      
 //modify the video's parametres      
ext.SetVideoParams = function(width,height,quality) {
   try{
       console.log("setting video streaming parametres")
       streaming_width=width;
       streaming_height=height;
       streaming_quality=quality;
       }catch(err){console.log("Unable to set streaming parametres")};
      }; 
      
 //stream gmapping process
 //must first launch: 
 //"roslaunch minilab_description minilab_state_publisher.launch"
 //and: "roslaunch minilab_navigation gmapping.launch"                      
ext.StreamMap= function(adress,port) {
   try{ 
      var popup_window=window.open("","Streaming Map scaning","resizable=yes,toolbar=no,titlebar=no,location=no,menubar=no,scrollbars=no,top=700,left=20,width=450,height=300");
var output=
"<!DOCTYPE html>"+
"<html>"+
"<head>"+
' <meta charset="utf-8"/>'+
'<script type="text/javascript" src="http://cdn.robotwebtools.org/EaselJS/current/easeljs.min.js"></script>'+
'<script type="text/javascript" src="http://cdn.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js"></script>'+
'<script type="text/javascript" src="http://cdn.robotwebtools.org/roslibjs/current/roslib.min.js"></script>'+
'<script type="text/javascript" src="http://cdn.robotwebtools.org/ros2djs/current/ros2d.min.js"></script>'+
'<script id="JavaSript">\n'+
       'function init() {\n'+
    '// Connect to ROS.\n'+
    'var ros = new ROSLIB.Ros({\n'+
      "url : 'ws://"+adress+":"+port+"'\n"+
    '});\n'+
    '// Create the main viewer.\n'+
    'var viewer = new ROS2D.Viewer({\n'+
      "divID : 'map',\n"+
      'width : '+map_width+',\n'+
      'height :'+map_height+',\n'+
    '});\n'+
    '// Setup the map client.\n'+
    'var gridClient = new ROS2D.OccupancyGridClient({\n'+
      'ros : ros,\n'+
      'rootObject : viewer.scene,\n'+
      "topic : '/map'\n"+
    '});\n'+
    '// Scale the canvas to fit to the map\n'+
    "gridClient.on('change', function(){\n"+
      'viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);\n'+
      'viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);\n'+
       'displayPoseMarker();\n'+
   '});\n'+
   '// ----------------------------------------------------------------------\n'+
      '// Showing the pose on the map\n'+
      '// ----------------------------------------------------------------------\n'+
      'function displayPoseMarker() {\n'+
        '// Create a marker representing the robot.\n'+
        'var robotMarker = new ROS2D.NavigationArrow({\n'+
          'size : 12,\n'+
          'strokeSize : 1,\n'+
          'fillColor : createjs.Graphics.getRGB(255, 128, 0, 0.66),\n'+
          'pulse : true\n'+
        '});\n'+
        'robotMarker.visible = false;\n'+
        '// Add the marker to the 2D scene.\n'+
        'gridClient.rootObject.addChild(robotMarker);\n'+
        'var initScaleSet = false;\n'+
        "// Subscribe to the robot's pose updates.\n"+
        'var poseListener = new ROSLIB.Topic({\n'+
          'ros : ros,\n'+
          "name : '/odom',\n"+
          "messageType : 'nav_msgs/Odometry',\n"+
          'throttle_rate : 100\n'+
        '});\n'+
        'poseListener.subscribe(function(pose) {\n'+
          "// Orientate the marker based on the robot's pose.\n"+
          "robotMarker.x = pose.pose.pose.position.x;\n"+
          "robotMarker.y = -pose.pose.pose.position.y;\n"+
          'if (!initScaleSet) {\n'+
            'robotMarker.scaleX = 1.0 / viewer.scene.scaleX;\n'+
            'robotMarker.scaleY = 1.0 / viewer.scene.scaleY;\n'+
            'initScaleSet = true;\n'+
          '}\n'+
          'robotMarker.rotation = viewer.scene.rosQuaternionToGlobalTheta(pose.pose.pose.orientation);\n'+
          'robotMarker.visible = true;\n'+
        '});\n'+
      '}\n'+
  '}\n'+
 "</script>\n"+
 '<script type="text/javascript">\n'+
//'setTimeout(function() { location.reload(false);},'+map_refresh*1000+');\n'+
'</script>\n'+
"</head>\n"+
'<body onload="init()">\n'+
'<div id="map"></div>\n'+
"</body>\n"+
"</html>\n";
popup_window.document.open();
popup_window.document.write(output);      
popup_window.document.close();
//setTimeout(function() { popup_window.document.location.reload(false);},map_refresh*1000);
   }catch(err){console.log("Unable to stream map")};
} ;   

//modify the map view resolution and refresh rate
ext.SetMapParams = function(width,height,refresh) {
   try{
  map_width=width;
  map_height=height;
  map_refresh=refresh;
 }catch(err){console.log("Unable to set Map streaming settings")};
      };  
      
//getting the laser parametres
//subscription to the topic: /scan
ext.GetLaserstartAngle = function() {
   try{
   var laser_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/scan',
    messageType : 'sensor_msgs/LaserScan'
  });
  
  laser_listener.subscribe(function(message) {
    console.log('Received /scan/angle_min:'+message.angle_min);
    laser_min_angle=message.angle_min;
    laser_listener.unsubscribe();
  });
  return laser_min_angle;
 }catch(err){console.log("Unable to Get Laser Angle min")};
      };  
      
ext.GetLaserendAngle = function() {
   try{ 
   var laser_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/scan',
    messageType : 'sensor_msgs/LaserScan'
  });
  laser_listener.subscribe(function(message) {
    console.log('Received /scan/angle_max:'+message.angle_max);
    laser_max_angle=message.angle_max;
    laser_listener.unsubscribe();
  });
  return laser_max_angle;
 }catch(err){console.log("Unable to Get Laser Angle max")};
      };  

ext.GetLaserdistAngle = function() {
   try{ 
   var laser_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/scan',
    messageType : 'sensor_msgs/LaserScan'
  });
  laser_listener.subscribe(function(message) {
    console.log('Received /scan/angle_increment:'+message.angle_increment);
    laser_inc_angle=message.angle_increment;
    laser_listener.unsubscribe();
  });
  return laser_inc_angle;
 }catch(err){console.log("Unable to Get Laser angle_increment")};
      };       
      
ext.GetLaserMesTime = function() {
   try{ 
   var laser_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/scan',
    messageType : 'sensor_msgs/LaserScan'
  });
  laser_listener.subscribe(function(message) {
    console.log('Received /scan/time_increment:'+message.time_increment);
    laser_time_inc=message.time_increment;
    laser_listener.unsubscribe();
  });
  return laser_time_inc; 
 }catch(err){console.log("Unable to Get Laser time_increment")};
      };             
      
ext.GetLaserScanTime = function() {
   try{ 
   var laser_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/scan',
    messageType : 'sensor_msgs/LaserScan'
  });
  laser_listener.subscribe(function(message) {
    console.log('Received /scan/tscan_time:'+message.scan_time);
    laser_scan_time=message.scan_time;
    laser_listener.unsubscribe();
  });
  return laser_scan_time;
 }catch(err){console.log("Unable to Get Laser scan_time")};
      };      
      
ext.GetLaserMaxRange = function() {
   try{ 
   var laser_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/scan',
    messageType : 'sensor_msgs/LaserScan'
  });
  laser_listener.subscribe(function(message) {
    console.log('Received /scan/range_max:'+message.range_max);
    laser_max_range=message.range_max;
    laser_listener.unsubscribe();
  });
  return laser_max_range;
 }catch(err){console.log("Unable to Get Laser range_max")};
      };            
      
ext.GetLaserMinRange = function() {
   try{ 
   var laser_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/scan',
    messageType : 'sensor_msgs/LaserScan'
  });
  laser_listener.subscribe(function(message) {
    console.log('Received /scan/range_min:'+message.range_min);
    laser_min_range=message.range_min;
    laser_listener.unsubscribe();
  });
  return laser_min_range;
 }catch(err){console.log("Unable to Get Laser range_min")};
      };             
//getting the range of a specified angle of the laser      
ext.GetLaserAngleRange = function(angle) {
   try{ 
   var laser_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/scan',
    messageType : 'sensor_msgs/LaserScan'
  });
  
  laser_listener.subscribe(function(message) {
    var angle_index=Math.ceil(Math.abs((message.angle_max+angle)/message.angle_increment));
    console.log('Received /scan/ranges['+angle_index+']:'+message.ranges[angle_index]);
    laser_angle_range=message.ranges[angle_index]
    laser_listener.unsubscribe();
  });
  return laser_angle_range;
 }catch(err){console.log("Unable to Get Laser ranges")};
      };  
      
    // Block and block menu descriptions
    var descriptor = {
        blocks: [
            // Block type, block name, function name
            ["h", "When connected to Mini-Lab","TestConnection"],
            ["", "Connect to Mini-Lab Adress: %s Port: %n","RosConnection","localhost","9090"],
            ["", "Move Mini-Lab Direction %m.direction_menu Speed %n", "MoveRobot","Forward",0.2],
            ["", "Stop Mini-Lab", "StopRobot"],
            ["","Stream video Adress: %s Port:%n ","StreamVideo","localhost","8080"],
            ["","Set video params: %n  X  %n ,Quality:  %n","SetVideoParams","400","200","90"],
            ["","Stream Map Scaning Adress: %s Port:%n ","StreamMap","localhost","9090"],
            ["","Set Map params: %n  X  %n ,Refresh_rate:  %n [Sec]","SetMapParams","430","280","1"],
            ["r","Mini-Lab:position.X","GetMinilabPositionX"],
            ["r","Mini-Lab:position.Y","GetMinilabPositionY"],
            ["r","Mini-Lab:position.Z","GetMinilabPositionZ"],
            ["r","Mini-Lab:rotation.X","GetMinilabRotationX"],
            ["r","Mini-Lab:rotation.Y","GetMinilabRotationY"],
            ["r","Mini-Lab:rotation.Z","GetMinilabRotationZ"],
            ["r","Mini-Lab:rotation.W","GetMinilabRotationW"],
            ["r","Laser:start angle of the scan [rd]","GetLaserstartAngle"],
            ["r","Laser end angle of the scan [rd]","GetLaserendAngle"],
            ["r","Laser angular distance between measurements [rd]","GetLaserdistAngle"],
            ["r","Laser:time between measurements [seconds]","GetLaserMesTime"],
            ["r","Laser:time between scans [seconds]","GetLaserScanTime"],
            ["r","Laser:minimum range value [m]","GetLaserMinRange"],
            ["r","Laser:maximum range value [m]","GetLaserMaxRange"],
            ["r","Laser:Get range of the angle %n rd [m]","GetLaserAngleRange","0"],
            
        ],
        menus: {
            "direction_menu":["Forward","Backward","Right","Left"]
        },
    };

    // Register the extension
    ScratchExtensions.register('Enova Mini-Lab', descriptor, ext);
})({});
