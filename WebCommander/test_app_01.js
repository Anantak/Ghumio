var express = require('express');
var app = express();
var server = require('http').Server(app);
var io = require('socket.io')(server);

// ZMQ transport, subscribers and publishers
var zmq = require('zmq');

// Status messages subscriber - these messages come from the ProgramCommander
var zmq_socket_sub = zmq.socket('sub');
zmq_socket_sub.identity = 'subscriber' + process.pid;
zmq_socket_sub.connect('tcp://localhost:6500');
zmq_socket_sub.subscribe('ComponentStatus ');

// Status query subscriber. This is how ProgramCommander gets the status queries to WebCommander
var zmq_socket_status = zmq.socket('sub');
zmq_socket_status.identity = 'subscriber' + process.pid + '.2';
zmq_socket_status.connect('tcp://localhost:6500');
zmq_socket_status.subscribe('WebCommander ');

//////// Sensor data subscribers - Temporary design before we build a better one //////////
// Cam1 Apriltag subscriber
var zmq_socket_Cam1April = zmq.socket('sub');
zmq_socket_Cam1April.identity = 'subscriber' + process.pid + '.01';
zmq_socket_Cam1April.connect('tcp://localhost:10504');
zmq_socket_Cam1April.subscribe('Cam1April ');
///////////////////////////////////////////////////////////////////////////////////////////

// WebCommander's publisher
var zmq_socket_pub = zmq.socket('pub');
var zmq_port = 'tcp://*:5550';
zmq_socket_pub.identity = 'publisher' + process.pid;
zmq_socket_pub.bind(zmq_port, function(err) {
  if (err) throw err;
  console.log('Bound to zmq socket on port ' + zmq_port);
});


var fs = require('fs');
var panels_location = '../ProgramCommander/src/ComponentCommander/ComponentPanels';
var files = fs.readdirSync(panels_location);
var all_panels_str = "";
for (var i in files) {
  var file_str = fs.readFileSync(panels_location+"/"+files[i], "utf8");
  all_panels_str = all_panels_str + file_str;
  console.log('File: ' + file_str);
}

var swig = require('swig');
var tpl = swig.compileFile('./Templates/index_test_00.html');
var page_rendering = tpl({Panels: all_panels_str});

var start_time = new Date().getTime();
start_time *= 1000;
console.log('Start time (us) = ' + start_time);

var exit_command_str = "COMMAND WebCommander exit";
var process_id = process.pid;

server.listen(3000);

app.use('/javascript', express.static(__dirname + '/javascript'));
app.use('/stylesheet', express.static(__dirname + '/stylesheet'));

app.get('/', function (req, res) {
  res.send(page_rendering)
});

io.on('connection', function (socket) {
  console.log('Incoming connection');

  // incoming messages
  socket.on('fromBrowser', function(data) {
    console.log(data);
    zmq_socket_pub.send(data.msg);
    //console.log(zmq_socket_pub.identity + ': sent ' + data.msg.toString());
    console.log('Out: ' + data.msg.toString());
    // exit if WebCommander has been asked to exit
    if (data.msg == exit_command_str) {
      console.log("Exiting WebCommander");
      process.exit(0);
    }
  });

  // outgoing messages coming in via zmq
  zmq_socket_sub.on('message', function(data) {
    var data_str = data.toString();
    //console.log(zmq_socket_sub.identity + ': received data ' + data_str);
    console.log('In: ' + data_str);
    var json_msg = data_str.substring(16, data_str.length);
    var msg_data = JSON.parse(json_msg);
    //console.log(json_msg);
    socket.emit('fromServer', msg_data);
  });
});

// StatusQuery messages from ProgramCommander
zmq_socket_status.on('message', function(data) {
  var data_str = data.toString();
  //console.log(zmq_socket_status.identity + ': received data ' + data_str);
  var reply_str = data_str.replace('WebCommander','StatusReply');
  var current_time = new Date().getTime();
  current_time *= 1000;
  var up_time = current_time - start_time;
  // Creating a protobuf text format message by hand - to avoid using a node.js protobuf port
  reply_str = reply_str + 'reply_time: ' + current_time +
              '\nquery_reply_delay: 000000\nup_time: ' + up_time +
              '\nprocess_id: ' + process_id;
  //console.log('Sending Status reply = ' + reply_str);
  zmq_socket_pub.send(reply_str);
});

// StatusReply query_time: 1418511532483427
// reply_time: 1418511532583870
// query_reply_delay: 100443
// up_time: 31195029

//////// Sensor data subscribers - Temporary design before we build a better one //////////
// AprilTag messages from Cam1April sensor
zmq_socket_Cam1April.on('message', function(data) {
  var current_time = new Date().getTime();
  current_time *= 1000;
  console.log('Cam1April msg at ' + current_time + ' len = ' + data.length);
});
///////////////////////////////////////////////////////////////////////////////////////////

// Example code
  //var timer01 = setInterval(function() {
  //  socket.emit('news', { hello: new Date().toString() });  
  //},1000);