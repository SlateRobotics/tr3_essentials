#!/usr/bin/env node

var http = require('http');
var bodyParser = require('body-parser');
var express = require('express');

var app = express();

app.use(function (req, res, next) {
	console.log(req.method + req.path);
	next();
});

app.use('/', require('./server/router'));

var port = process.env.TR_OS_PORT || 8080;
var httpServer = http.createServer(app).listen(port);

console.log("Server hosted on port " + port);

var io = require('socket.io')(httpServer);
io.on('connection', function (socket) {
  var state = {a0: 0.0, a1: 0.0, a2: 0.0, a3: 0.0, a4: 0.0, h0: 0.0, h1: 0.0, g0: 0.0};

  function sendState() {
    socket.emit('robot_state', state);
    for (var j in state) {
      state[j] += 1
    }
  }

  setInterval(sendState, 50);
});
