if (!tr) tr = {};
if (!tr.data) tr.data = {};

tr.data.socket = '';
tr.data.robotState = {};

tr.data.setup = function() {
  tr.data.socket = io('http://localhost:80/');
  tr.data.socket.on('robot_state', function(data) {
    tr.data.robotState = data;
  });
}

tr.data.request = function(opts) {

};
