if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.txtPID = function(controller, type) {
  return {
    type: "container",
    size: {
      w: 1 / 4,
      h: 45
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
          var limitIdx = 0;
          if (controller == "position") {
            pid = "pid_pos";
            limitIdx = 0;
          } else if (controller == "velocity") {
            pid = "pid_vel";
            limitIdx = 2;
          } else if (controller == "torque") {
            pid = "pid_trq";
            limitIdx = 4;
          }

          if (tr.data.joints[id][pid]) {
            if (type == "P") {
              this.text = (tr.data.joints[id][pid][0] || 0.0).toFixed(2);
            } else if (type == "I") {
              this.text = (tr.data.joints[id][pid][1] || 0.0).toFixed(2);
            } else if (type == "D") {
              this.text = (tr.data.joints[id][pid][2] || 0.0).toFixed(2);
            } else if (type == "IC") {
              this.text = (tr.data.joints[id][pid][3] || 0.0).toFixed(2);
            } else if (type == "FF") {
              this.text = (tr.data.joints[id][pid][4] || 0.0).toFixed(2);
            } else if (type == "MIN") {
              this.text = (tr.data.joints[id].limit[limitIdx] || 0.0).toFixed(2);
            } else if (type == "MAX") {
              this.text = (tr.data.joints[id].limit[limitIdx + 1] || 0.0).toFixed(2);
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
