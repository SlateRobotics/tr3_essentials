if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.programBtn_Pause = function() {
  return {
    type: "container",
    size: {
      w: 1 / 3,
      h: 40
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        //Do program play
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "II", // Right symbol
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
