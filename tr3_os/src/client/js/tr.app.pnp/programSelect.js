if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.programSelect = function() {
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
        //Do program Settings
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        id: "progselect",
        type: "select",
        size: {
          w: 1,
          h: 40
        },
        padding: 5,
        options: ["Did Not Load", ],
        textSize: 22,
        onChange: function(val) {
          var app = this.getApp();
        //  app.config.changeProgram(val);
        },
      }],
    }]
  }
}
