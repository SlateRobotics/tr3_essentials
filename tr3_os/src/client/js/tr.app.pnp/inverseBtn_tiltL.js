if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.inverseBtn_tiltL = function(xyz) {
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

        if (xyz == x) {

        }
        if (xyz == y) {

        }
        if (xyz == z) {

        }
        //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "◣", // Add symbol
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
