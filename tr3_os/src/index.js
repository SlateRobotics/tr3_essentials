#!/usr/bin/env node

var http = require('http');
var bodyParser = require('body-parser');
var express = require('express');
var rosnodejs = require('rosnodejs');
var stdMsgs = rosnodejs.require('std_msgs');

var app = express();

app.use(function (req, res, next) {
	console.log(req.method + req.path);
	next();
});

app.use('/', require('./server/router'));

var port = process.env.TR_OS_PORT || 8080;
var httpServer = http.createServer(app).listen(port);

console.log("Server hosted on port " + port);

var rostopics = [
  { name: "/tr3/stop", type: "std_msgs/Bool"},
  { name: "/tr3/shutdown", type: "std_msgs/Bool"},
  { name: "/tr3/powerup", type: "std_msgs/Bool"},
  { name: "/tr3/joints/a0/control/position", type: "std_msgs/Float64"},
  { name: "/tr3/joints/a1/control/position", type: "std_msgs/Float64"},
  { name: "/tr3/joints/a2/control/position", type: "std_msgs/Float64"},
  { name: "/tr3/joints/a3/control/position", type: "std_msgs/Float64"},
  { name: "/tr3/joints/a4/control/position", type: "std_msgs/Float64"},
  { name: "/tr3/joints/h0/control/position", type: "std_msgs/Float64"},
  { name: "/tr3/joints/h1/control/position", type: "std_msgs/Float64"},
  { name: "/tr3/joints/b0/control/position", type: "std_msgs/Float64"},
  { name: "/tr3/joints/b1/control/position", type: "std_msgs/Float64"},
  { name: "/tr3/joints/a0/control/effort", type: "std_msgs/Float64"},
  { name: "/tr3/joints/a1/control/effort", type: "std_msgs/Float64"},
  { name: "/tr3/joints/a2/control/effort", type: "std_msgs/Float64"},
  { name: "/tr3/joints/a3/control/effort", type: "std_msgs/Float64"},
  { name: "/tr3/joints/a4/control/effort", type: "std_msgs/Float64"},
  { name: "/tr3/joints/h0/control/effort", type: "std_msgs/Float64"},
  { name: "/tr3/joints/h1/control/effort", type: "std_msgs/Float64"},
  { name: "/tr3/joints/b0/control/effort", type: "std_msgs/Float64"},
  { name: "/tr3/joints/b1/control/effort", type: "std_msgs/Float64"},
  { name: "/tr3/joints/a0/mode", type: "std_msgs/UInt8"},
  { name: "/tr3/joints/a1/mode", type: "std_msgs/UInt8"},
  { name: "/tr3/joints/a2/mode", type: "std_msgs/UInt8"},
  { name: "/tr3/joints/a3/mode", type: "std_msgs/UInt8"},
  { name: "/tr3/joints/a4/mode", type: "std_msgs/UInt8"},
  { name: "/tr3/joints/h0/mode", type: "std_msgs/UInt8"},
  { name: "/tr3/joints/h1/mode", type: "std_msgs/UInt8"},
  { name: "/tr3/joints/b0/mode", type: "std_msgs/UInt8"},
  { name: "/tr3/joints/b1/mode", type: "std_msgs/UInt8"},
  { name: "/tr3/joints/a0/reset", type: "std_msgs/Bool"},
  { name: "/tr3/joints/a1/reset", type: "std_msgs/Bool"},
  { name: "/tr3/joints/a2/reset", type: "std_msgs/Bool"},
  { name: "/tr3/joints/a3/reset", type: "std_msgs/Bool"},
  { name: "/tr3/joints/a4/reset", type: "std_msgs/Bool"},
  { name: "/tr3/joints/h0/reset", type: "std_msgs/Bool"},
  { name: "/tr3/joints/h1/reset", type: "std_msgs/Bool"},
  { name: "/tr3/joints/b0/reset", type: "std_msgs/Bool"},
  { name: "/tr3/joints/b1/reset", type: "std_msgs/Bool"},
  { name: "/tr3/joints/a0/flip", type: "std_msgs/Bool"},
  { name: "/tr3/joints/a1/flip", type: "std_msgs/Bool"},
  { name: "/tr3/joints/a2/flip", type: "std_msgs/Bool"},
  { name: "/tr3/joints/a3/flip", type: "std_msgs/Bool"},
  { name: "/tr3/joints/a4/flip", type: "std_msgs/Bool"},
  { name: "/tr3/joints/h0/flip", type: "std_msgs/Bool"},
  { name: "/tr3/joints/h1/flip", type: "std_msgs/Bool"},
  { name: "/tr3/joints/b0/flip", type: "std_msgs/Bool"},
  { name: "/tr3/joints/b1/flip", type: "std_msgs/Bool"},
  { name: "/tr3/joints/a0/pid", type: "std_msgs/Float32MultiArray"},
  { name: "/tr3/joints/a1/pid", type: "std_msgs/Float32MultiArray"},
  { name: "/tr3/joints/a2/pid", type: "std_msgs/Float32MultiArray"},
  { name: "/tr3/joints/a3/pid", type: "std_msgs/Float32MultiArray"},
  { name: "/tr3/joints/a4/pid", type: "std_msgs/Float32MultiArray"},
  { name: "/tr3/joints/h0/pid", type: "std_msgs/Float32MultiArray"},
  { name: "/tr3/joints/h1/pid", type: "std_msgs/Float32MultiArray"},
  { name: "/tr3/joints/b0/pid", type: "std_msgs/Float32MultiArray"},
  { name: "/tr3/joints/b1/pid", type: "std_msgs/Float32MultiArray"},
]

var io = require('socket.io')(httpServer);
io.on('connection', function (socket) {
  rosnodejs.initNode('/tr3_os').then(function () {
    var nh = rosnodejs.nh;
    nh.subscribe('/tr3/state', 'sensor_msgs/JointState', function (msg) {
      socket.emit('/tr3/state', msg);
    });

    for (var i = 0; i < rostopics.length; i++) {
      const rt = nh.advertise(rostopics[i].name, rostopics[i].type);
      socket.on(rostopics[i].name, function (data) {
        if (rt._type == "std_msgs/Float32MultiArray") {
          var d = new stdMsgs.msg.Float32MultiArray({ data: data });
          rt.publish(d)
        } else {
          rt.publish({ data: data });
        }
      }.bind(rt))
    }
  });
});
