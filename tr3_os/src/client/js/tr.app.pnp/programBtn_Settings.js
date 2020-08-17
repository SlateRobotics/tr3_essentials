if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};
var p = tr.controls.pnp2.program_Tools;

tr.controls.pnp2.programBtn_Settings = function() {
  return {
    type: "container",
    size: {
      w: 1 / 4,
      h: 50
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        var app = this.getApp().config;
        //Do program Settings
        //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "S", // ⚙ Gear symbol Not Working
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
