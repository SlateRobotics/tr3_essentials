if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.inverseBtn_Blnk = function() {
  var c = tr.controls.pnp2;

  var children = [];
  children.push(c.render());

  return {
    type: "container",
    size: {
      w: 1 / 5,
      h: 1 / 6,
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
