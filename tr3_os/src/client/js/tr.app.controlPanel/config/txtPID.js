if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.txtPID = function(controller, type) {
  return {
    type: "container",
    size: {
      w: 1 / 4,
      h: 60
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: "txt-" + type,
        type: "text",
        text: "-",
        textSize: 18,
        onDraw: function () {
          var id = tr.controlPanel.state.currentActuator;

          var pid = "";
          if (controller == "position") {
            pid = "pid_pos";
          } else if (controller == "velocity") {
            pid = "pid_vel";
          } else if (controller == "torque") {
            pid = "pid_trq";
          }

          if (tr.data.joints[id][pid]) {
            if (type == "P") {
              this.text = tr.data.joints[id][pid][0].toFixed(3);
            } else if (type == "I") {
              this.text = tr.data.joints[id][pid][1].toFixed(3);
            } else if (type == "D") {
              this.text = tr.data.joints[id][pid][2].toFixed(3);
            }
          }
        },
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
