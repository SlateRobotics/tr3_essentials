if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.btnPID = function(id, type, lbl) {
  return {
    type: "container",
    size: {
      w: 1 / 18,
      h: 20
    },
    onClick: function() {
      var app = this.getApp();
      var page = app.getCurrentPage();

      var inc = 0;
      if (lbl == "▶") {
        inc += 0.1;
      } else if (lbl == "◀") {
        inc -= 0.1;
      }

      if (type == "P") {
        tr.data.joints[id].pid[0] += inc;
      } else if (type == "I") {
        tr.data.joints[id].pid[1] += inc;
      } else if (type == "D") {
        tr.data.joints[id].pid[2] += inc;
      }

      tr.data.socket.emit("/tr3/joints/" + id + "/pid/set", tr.data.joints[id].pid);
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: lbl,
        textSize: 14,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
