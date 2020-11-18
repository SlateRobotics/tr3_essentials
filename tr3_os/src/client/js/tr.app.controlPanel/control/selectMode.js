if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.selectMode = function() {
  return {
    type: "container",
    size: {
      w: 1.0,
      h: 70
    },

    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: "Mode",
        size: { w: 0.2, h: 1 },
        textSize: 20,
        padding: 10,
        align: {
          v: "CENTER",
          h: "LEFT"
        },
      }, {
        type: "select",
        id: "select-mode",
        size: { w: 0.7999, h: 1 },
        options: ["VOLTAGE", "TORQUE", "VELOCITY", "POSITION"],
        onChange: function(val) {
          var id = tr.controlPanel.state.currentActuator;
          var app = this.getApp();
          var page = app.getCurrentPage();

          tr.controlPanel.state.currentMode = val;

          var i = 0;
          if (val == "VOLTAGE") {
            i = 4;
            var slider = page.getChild("slider-control-voltage");
            slider.setval(0);

            page.getChild("slider-control-position").disabled = true;
            page.getChild("slider-control-velocity").disabled = true;
            page.getChild("slider-control-torque").disabled = true;
            page.getChild("slider-control-voltage").disabled = false;

          } else if (val == "TORQUE") {
            i = 3;
            var slider = page.getChild("slider-control-torque");
            slider.setval(0);

            page.getChild("slider-control-position").disabled = true;
            page.getChild("slider-control-velocity").disabled = true;
            page.getChild("slider-control-torque").disabled = false;
            page.getChild("slider-control-voltage").disabled = true;
          } else if (val == "POSITION") {
            i = 1;
            var state = tr.data.getState(id).position;
            var slider = page.getChild("slider-control-position");
            slider.setval(state / Math.PI / 2.0);

            page.getChild("slider-control-position").disabled = false;
            page.getChild("slider-control-velocity").disabled = true;
            page.getChild("slider-control-torque").disabled = true;
            page.getChild("slider-control-voltage").disabled = true;
          } else if (val == "VELOCITY") {
            i = 2;
            var slider = page.getChild("slider-control-velocity");
            slider.setval(0);

            page.getChild("slider-control-position").disabled = true;
            page.getChild("slider-control-velocity").disabled = false;
            page.getChild("slider-control-torque").disabled = true;
            page.getChild("slider-control-voltage").disabled = true;
          }

          tr.data.socket.emit("/tr3/" + id + "/mode", i);
        },
        Size: {
          w: 1,
          h: 20
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
