if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.waypointBtn_Previous = function() {
  return {
    type: "container",
    size: {
      w: 1 / 5,
      h: 50
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        //Do waypoint previous
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "◀", // Left symbol
        textSize: 24,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
