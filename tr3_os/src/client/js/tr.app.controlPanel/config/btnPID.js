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
      if (controller == "position") {
        pid = "pid_pos";
      } else if (controller == "velocity") {
        pid = "pid_vel";
      } else if (controller == "torque") {
        pid = "pid_trq";
      }

      var inc = 0;
      if (lbl == "▶") {
	if (pid == "pid_trq") {
          inc += 0.005;
	} else {
          inc += 0.1;
        }
      } else if (lbl == "◀") {
	if (pid == "pid_trq") {
          inc -= 0.005;
	} else {
          inc -= 0.1;
	}
      }

      var id = tr.controlPanel.state.currentActuator;
      if (type == "P") {
        tr.data.joints[id][pid][0] += inc;
      } else if (type == "I") {
        tr.data.joints[id][pid][1] += inc;
      } else if (type == "D") {
        tr.data.joints[id][pid][2] += inc;
      }

      tr.data.socket.emit("/tr3/joints/" + id + "/" + pid + "/set", tr.data.joints[id][pid]);
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
