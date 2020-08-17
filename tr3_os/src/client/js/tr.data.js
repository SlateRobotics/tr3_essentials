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

tr.data.getState = function(aid) {
  for (var i = 0; i < tr.data.robotState.name.length; i++) {
    if (tr.data.robotState.name[i] == aid) {
      return {
        position: tr.data.robotState.position[i],
        velocity: tr.data.robotState.velocity[i],
        effort: tr.data.robotState.effort[i]
      }
    }
  }

  return {};
}
