#!/usr/bin/env node

var http = require('http');
var bodyParser = require('body-parser');
var express = require('express');
var rosnodejs = require('rosnodejs');
var stdMsgs = rosnodejs.require('std_msgs');
var geometryMsgs = rosnodejs.require('geometry_msgs');
var actionlibMsgs = rosnodejs.require('actionlib_msgs');
var tr3Msgs = rosnodejs.require('tr3_msgs');
var qte = require('quaternion-to-euler');
var nj = require('@aas395/numjs');

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
  { name: "/tr3/base/diff/cmd_vel", type: "geometry_msgs/Twist"},
  { name: "/move_base/cancel", type: "actionlib_msgs/GoalID"},
  { name: "/move_base_simple/goal", type: "geometry_msgs/PoseStamped"},
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
    var srvForwardIk = nh.serviceClient('/forward_ik', tr3Msgs.srv.ForwardIK);
    var srvInverseIk = nh.serviceClient('/inverse_ik', tr3Msgs.srv.InverseIK);

    nh.subscribe('/tr3/state', 'sensor_msgs/JointState', function (msg) {
      socket.emit('/tr3/state', msg);
    });

    socket.on('/forward-ik', function (msg) {
        var req = new tr3Msgs.srv.ForwardIK.Request();
      	req.state.name = msg.name;
      	req.state.position = msg.position;
      	req.state.velocity = msg.velocity;
      	req.state.effort = msg.effort;
      	srvForwardIk.call(req).then(function (res) {
          socket.emit('/forward-ik-' + msg.id, res);
        }).catch(function (err) {
          console.log(err);
          socket.emit('/forward-ik-' + msg.id, { err: true });
        });
    });

    socket.on('/inverse-ik', function (msg) {
        var req = new tr3Msgs.srv.InverseIK.Request();
      	req.pose.position = msg.position;
      	req.pose.orientation = msg.orientation;
      	srvInverseIk.call(req).then(function (res) {
          socket.emit('/inverse-ik-' + msg.id, res);
        }).catch(function (err) {
          console.log(err);
          socket.emit('/inverse-ik-' + msg.id, { err: true });
        });
    });

    nh.subscribe('/tr3/depth/scaled', 'std_msgs/Int32MultiArray', function (msg) {
      var result = [];
      for (var i = 0; i < msg.data.length; i += 4) {
        var n = msg.data[i+0];
        var x = msg.data[i+1] / 1000000.0;
        var y = msg.data[i+2] / 1000000.0;
        var z = msg.data[i+3] / 1000000.0;

        if (n == 1) {
          //result.push(NaN);
          //result.push(NaN);
          //result.push(NaN);
        } else {
          result.push(x);
          result.push(y);
          result.push(z);
        }
      }

      //socket.emit('/tr3/depth', result);
    });

    nh.subscribe('/tr3/lidar', 'sensor_msgs/LaserScan', function (msg) {
      socket.emit('/tr3/lidar', {
        angle_increment: msg.angle_increment,
        angle_min: msg.angle_min,
        angle_max: msg.angle_max,
        ranges: msg.ranges
      });
    });

    nh.subscribe('/move_base/status' ,'actionlib_msgs/GoalStatusArray', function (msg) {
      socket.emit('/move_base/status', msg);
    });

    nh.subscribe('/tr3/base/odom' ,'nav_msgs/Odometry', function (msg) {
      var q = msg.pose.pose.orientation;
      var euler = qte([q.w, q.x, q.y, q.z]);
      socket.emit('/tr3/base/odom', {
        position: msg.pose.pose.position,
        orientation: {
          x: euler[0],
          y: euler[1],
          z: euler[2],
        }
      });
    });

    // need to properly scale this down, biggest area for optimizations
    // UI render is fairly quick under ~1000 points
    nh.subscribe('/map', 'nav_msgs/OccupancyGrid', function (msg) {
      var result = [];
      var r = [];

      for (var i = 0; i < msg.data.length; i++) {
        if (msg.data[i] == -1) {
          msg.data[i] = 0;
        } else {
          msg.data[i] = Math.floor(msg.data[i] * (255 / 100));
        }
      }

      var m = nj.array(msg.data);
      m = m.reshape(msg.info.height, msg.info.width);
      r = nj.images.resize(m, 200, 200);

      var res = msg.info.resolution * (msg.info.width / r.shape[1]);

      for (var w = 0; w < r.shape[1]; w++) {
        for (var h = 0; h < r.shape[0]; h++) {
          var d = r.get(h, w);
          if (d > 50) {
            result.push(w * res + res / 2.0 + msg.info.origin.position.x);
            result.push(h * res + res / 2.0 + msg.info.origin.position.y);
          }
        }
      }

      socket.emit('/map', result);
    });

    for (var i = 0; i < rostopics.length; i++) {
      const rt = nh.advertise(rostopics[i].name, rostopics[i].type);
      socket.on(rostopics[i].name, function (data) {
        if (rt._type == "std_msgs/Float32MultiArray") {
          var d = new stdMsgs.msg.Float32MultiArray({ data: data });
          rt.publish(d)
        } else if (rt._type == "geometry_msgs/Twist") {
          var d = new geometryMsgs.msg.Twist(data);
          rt.publish(d)
        } else if (rt._type == "geometry_msgs/PoseStamped") {
          var d = new geometryMsgs.msg.PoseStamped(data);
          rt.publish(d)
        } else if (rt._type == "actionlib_msgs/GoalID") {
          var d = new actionlibMsgs.msg.GoalID(data);
          rt.publish(d)
        } else {
          rt.publish({ data: data });
        }
      }.bind(rt))
    }
  });
});
