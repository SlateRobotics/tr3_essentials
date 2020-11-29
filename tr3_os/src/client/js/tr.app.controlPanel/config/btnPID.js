if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.btnPID = function(controller, type, lbl) {
  return {
    type: "container",
    size: {
      w: 1/4,
      h: 60
    },
    onClick: function() {
      var app = this.getApp();
      var page = app.getCurrentPage();

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

      var inc = 0;
      if (lbl == "▶") {
        if (pid == "pid_trq") {
          if (type == "MIN" || type == "MAX") {
            inc += 1.0;
          } else {
            inc += 0.01;
          }
        } else {
          if (type == "D") {
            inc += 0.01;
          } else {
            inc += 0.1;
          }
        }
      } else if (lbl == "◀") {
        if (pid == "pid_trq") {
          if (type == "MIN" || type == "MAX") {
            inc -= 1.0;
          } else {
            inc -= 0.01;
          }
        } else {
          if (type == "D") {
            inc -= 0.01;
          } else {
            inc -= 0.1;
          }
        }
      }

      // round number to 2 decimal places
      // ensures that 0.0 is sent to joint vs. -0.00000000001
      function r (v) {
        return Math.round(v * 100.0) / 100.0;
      }

      var id = tr.controlPanel.state.currentActuator;
      if (type == "P") {
        tr.data.joints[id][pid][0] = r(tr.data.joints[id][pid][0] + inc);
        tr.data.socket.emit("/tr3/" + id + "/" + pid + "/set", tr.data.joints[id][pid]);
      } else if (type == "I") {
        tr.data.joints[id][pid][1] = r(tr.data.joints[id][pid][1] + inc);
        tr.data.socket.emit("/tr3/" + id + "/" + pid + "/set", tr.data.joints[id][pid]);
      } else if (type == "D") {
        tr.data.joints[id][pid][2] = r(tr.data.joints[id][pid][2] + inc);
        tr.data.socket.emit("/tr3/" + id + "/" + pid + "/set", tr.data.joints[id][pid]);
      } else if (type == "MIN") {
        tr.data.joints[id].limit[limitIdx] = r(tr.data.joints[id].limit[limitIdx] + inc);
        tr.data.socket.emit("/tr3/" + id + "/limit/" + controller + "/min", tr.data.joints[id].limit[limitIdx]);
      } else if (type == "MAX") {
        tr.data.joints[id].limit[limitIdx + 1] = r(tr.data.joints[id].limit[limitIdx + 1] + inc)
        tr.data.socket.emit("/tr3/" + id + "/limit/" + controller + "/max", tr.data.joints[id].limit[limitIdx + 1]);
      }

    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: lbl,
        textSize: 18,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
