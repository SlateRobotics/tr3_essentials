if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.inverseBtn_Display = function(lbl) {
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
      },
      children: [{
        id: "lbl-inverse-" + lbl,
        type: "text",
        text: lbl,
        textSize: 12,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
