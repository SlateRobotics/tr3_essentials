if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.inverseBtn_zDown = function() {
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
        var app = this.getApp().config;
        var p = tr.controls.pnp2.program_Tools;
        p.inverseIk(app, { x: 0, y: 0, z: -0.05 });
      },
      children: [{
        type: "text",
        text: "▼", // Add symbol
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
