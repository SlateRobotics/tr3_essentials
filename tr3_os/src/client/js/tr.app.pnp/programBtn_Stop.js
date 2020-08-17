if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};
var p = tr.controls.pnp2.program_Tools;

tr.controls.pnp2.programBtn_Stop = function() {
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
          var app = this.getApp().config;
        p.programStop(app)
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "■", // Right symbol
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
