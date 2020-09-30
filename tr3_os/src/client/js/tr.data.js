if (!tr) tr = {};
if (!tr.data) tr.data = {};

tr.data.socket = '';
tr.data.robotState = {
  header: {},
  name: [],
  position: [],
  velocity: [],
  effort: []
};

tr.data.joints = {
  a0: { pid: [0, 0, 0] },
  a1: { pid: [0, 0, 0] },
  a2: { pid: [0, 0, 0] },
  a3: { pid: [0, 0, 0] },
  a4: { pid: [0, 0, 0] },
  g0: { pid: [0, 0, 0] },
  h0: { pid: [0, 0, 0] },
  h1: { pid: [0, 0, 0] },
  b0: { pid: [0, 0, 0] },
  b1: { pid: [0, 0, 0] }
}

tr.data.stopped = false;
tr.data.nav = {};

tr.data.depth = [];

tr.data.lidar = {
  angle_increment: 0,
  ranges: [],
}

tr.data.setup = function() {
  tr.data.socket = io();

  tr.data.socket.on('/tr3/state', function(data) {
    tr.data.robotState = data;
  });

  var aids = ["a0","a1","a2","a3","a4","g0","h0","h1","b0","b1"];
  for (var i = 0; i < aids.length; i++) {
    let aid = aids[i];
    tr.data.socket.on("/tr3/joints/" + aid + "/pid", function (data) {
      tr.data.joints[aid].pid = data;
    }.bind(aid))
  }

  tr.data.socket.on('/tr3/lidar', function(data) {
    tr.data.lidar = data;
  });

  tr.data.socket.on('/tr3/base/odom', function(data) {
    tr.data.odom = data;
  });

  tr.data.socket.on('/tr3/depth', function(data) {
    tr.data.depth = data;
  });

  tr.data.socket.on('/move_base/status', function(data) {
    var complete = true;
    for (var i = 0; i < data.status_list.length; i++) {
      var s = data.status_list[i].status;
      if (s == 0 || s == 1) {
        complete = false;
      }
    }
    tr.data.nav.status = data;
    tr.data.nav.complete = complete;
  });

  tr.data.socket.on('/map', function(data) {
    tr.data.map = data;
  });
}

tr.data.getState = function(aid, _state) {
  var s = tr.data.robotState;

  if (_state) {
    s = _state;
  }

  for (var i = 0; i < s.name.length; i++) {
    if (s.name[i] == aid) {
      return {
        position: s.position[i],
        velocity: s.velocity[i],
        effort: s.effort[i]
      }
    }
  }

  return {};
}

tr.data.getForwardIk = function (state, callback) {
  var id = Math.floor(Math.random() * (1000000000 + 1));

  tr.data.socket.on('/forward-ik-' + id, function (data) {
    callback(data.pose);
    tr.data.socket.off('/forward-ik-' + id);
  });

  var name = [];
  var position = [];
  var velocity = [];
  var effort = [];

  for (var property in state) {
    if (state.hasOwnProperty(property)) {
      name.push(property);
      position.push(state[property]);
      velocity.push(0);
      effort.push(0);
    }
  }

  tr.data.socket.emit('/forward-ik', {
    id: id,
    name: name,
    position: position,
    velocity: velocity,
    effort: effort
  });
}

tr.data.getInverseIk = function (pose, callback) {
  var id = Math.floor(Math.random() * (1000000000 + 1));

  tr.data.socket.on('/inverse-ik-' + id, function (data) {
    callback(data.state, data.err);
    tr.data.socket.off('/inverse-ik-' + id);
  });

  tr.data.socket.emit('/inverse-ik', {
    id: id,
    position: pose.position,
    orientation: pose.orientation
  });
}

tr.data.readDir = function (config) {
  var id = Math.floor(Math.random() * (1000000000 + 1));

  tr.data.socket.on('/readDir-' + id, function (data) {
    config.callback(data);
    tr.data.socket.off('/readDir-' + id);
  });

  tr.data.socket.emit('/readDir', {
    id: id,
    app: config.app,
  });
}

tr.data.deleteFile = function (config) {
  tr.data.socket.emit('/deleteFile', {
    app: config.app,
    fileName: config.fileName,
  });
}

tr.data.readFile = function (config) {
  var id = Math.floor(Math.random() * (1000000000 + 1));

  tr.data.socket.on('/readFile-' + id, function (data) {
    config.callback(data);
    tr.data.socket.off('/readFile-' + id);
  });

  tr.data.socket.emit('/readFile', {
    id: id,
    app: config.app,
    fileName: config.fileName,
  });
}

tr.data.writeFile = function (config) {
  tr.data.socket.emit("/writeFile", {
    app: config.app,
    fileName: config.fileName,
    contents: config.contents
  });
}
