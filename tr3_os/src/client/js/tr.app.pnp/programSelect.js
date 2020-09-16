if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};
var p = tr.controls.pnp2.program_Tools;

tr.controls.pnp2.programSelect = function() {
  return {
    type: "container",
    size: {
      w: 0.25,
      h: 1
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        //Do program Settings
        //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        id: "progselect",
        type: "select",
        size: {
          w: 1,
          h: 1
        },
        padding: 5,
        options: ["Did Not Load", ],
        textSize: 9,
        onChange: function(val) {
          var app = this.getApp().config;
          p.changeProgram(app, val);
        },
      }],
    }]
  }
}
