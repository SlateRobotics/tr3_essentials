if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.programBtn_Add = function() {
  var p = tr.controls.pnp2.program_Tools;
  
  return {
    type: "container",
    size: {
      w: 0.75/10,
      h: 1.0
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        var app = this.getApp().config;
        p.addProgram(app);
        //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "+", // Add symbol
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
