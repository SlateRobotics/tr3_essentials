if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.inverseBtn_Blnk = function() {
  return {
    type: "container",
    size: {
      w: 1 / 5,
      h: 35
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        //Do waypoint Add
        //tr.data.socket.emit(rostopic, value);
      }
    }]
  }
}
