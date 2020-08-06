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

tr.data.setup = function() {
  tr.data.socket = io('http://localhost:8080/');

  tr.data.socket.on('/tr3/state', function (data) {
    tr.data.robotState = data;
  });

  tr.data.socket.on('/camera/rgb/image_raw', function (data) {
    tr.data.cameraImage = data;
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
