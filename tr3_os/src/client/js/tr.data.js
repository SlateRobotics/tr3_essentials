if (!tr) tr = {};
if (!tr.data) tr.data = {};

tr.data.socket = '';
tr.data.joint_states = {
  header: {},
  name: [],
  position: [],
  velocity: [],
  effort: []
};

tr.data.power = false;

tr.data.joints = {};

tr.data.stopped = false;
tr.data.nav = {};

tr.data.depth = [];

tr.data.lidar = {
  angle_increment: 0,
  ranges: [],
}

tr.data.setup = function() {
  tr.data.socket = io();

  tr.data.socket.on('/tr3/joint_states', function(data) {
    tr.data.joint_states = data;
  });

  var aids = ["a0","a1","a2","a3","a4","g0","h0","h1","b0","b1"];
  for (var i = 0; i < aids.length; i++) {
    let aid = aids[i];
    tr.data.joints[aid] = {
      pid_pos_set: false,
      pid_pos: [0, 0, 0],
      pid_vel_set: false,
      pid_vel: [0, 0, 0],
      pid_trq_set: false,
      pid_trq: [0, 0, 0]
    }

    tr.data.socket.on("/tr3/" + aid + "/ip", function (msg) {
      tr.data.joints[aid].ip = msg.data;
    }.bind(aid))

    tr.data.socket.on("/tr3/" + aid + "/log", function (msg) {
      tr.data.joints[aid].log = msg.data;
    }.bind(aid))

    tr.data.socket.on("/tr3/" + aid + "/version", function (msg) {
      tr.data.joints[aid].version = msg.data;
    }.bind(aid))

    tr.data.socket.on("/tr3/" + aid + "/state", function (msg) {
      tr.data.joints[aid].state = msg;
    }.bind(aid))

    tr.data.socket.on("/tr3/" + aid + "/pid_pos", function (msg) {
      if (!tr.data.joints[aid].pid_pos_set) {
        tr.data.joints[aid].pid_pos = msg.data;
        tr.data.joints[aid].pid_pos_set = true;
      }
    }.bind(aid))

    tr.data.socket.on("/tr3/" + aid + "/pid_vel", function (msg) {
      if (!tr.data.joints[aid].pid_vel_set) {
        tr.data.joints[aid].pid_vel = msg.data;
        tr.data.joints[aid].pid_vel_set = true;
      }
    }.bind(aid))

    tr.data.socket.on("/tr3/" + aid + "/pid_trq", function (msg) {
      if (!tr.data.joints[aid].pid_trq_set) {
        tr.data.joints[aid].pid_trq = msg.data;
        tr.data.joints[aid].pid_trq_set = true;
      }
    }.bind(aid))
  }

  tr.data.socket.on("/tr3/power/state", function (msg) {
    tr.data.power = msg.data;
  });

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
  if (_state) {
    for (var i = 0; i < _state.name.length; i++) {
      if (_state.name[i] == aid) {
        return {
          position: _state.position[i],
          velocity: _state.velocity[i],
          effort: _state.effort[i]
        }
      }
    }
  } else {
    if (tr.data.joints[aid]) {
      return tr.data.joints[aid].state || {};
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
