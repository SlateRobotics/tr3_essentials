if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.inverseBtn_gClose = function() {
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
        p.getCurrentProgram(app).getCurrentWaypoint(app).setPosition("g0", 0);
        p.getCurrentProgram(app).save();
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
