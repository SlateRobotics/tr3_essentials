if (!tr) tr = {};
if (!tr.app) tr.app = {};
if (!tr.app.controlPanel) tr.app.controlPanel = {};

tr.app.controlPanel.selectMode = function(id) {
  return {
    type: "container",
    size: {
      w: 0.333,
      h: 25
    },

    children: [{
      type: "container",
      border: false,
      children: [{
        type: "select",
        id: "select-" + id,
        options: ["EFFORT", "BACKDRIVE", "SERVO"],
        onChange: function(val) {
          var app = this.getApp();
          var page = app.getCurrentPage();
          if (page) {
            var slider = page.getChild(id + "slider");
            if (slider) {
              if (val == "EFFORT") {
                slider.setval(0);
              } else if (val == "BACKDRIVE") {
                slider.setval(0);
              } else if (val == "SERVO") {
                var state = tr.data.getState(id).position;
                slider.setval(state / Math.PI / 2.0);
              }
            }
          }

          var i = 0;
          if (val == "EFFORT") {
            i = 0;
          } else if (val == "BACKDRIVE") {
            i = 1;
          } else if (val == "SERVO") {
            i = 2;
          }

          tr.data.socket.emit("/tr3/joints/" + id + "/mode", i);
        },
        Size: {
          w: 1,
          h: 25
        },
        textSize: 12,
        padding: 0,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
