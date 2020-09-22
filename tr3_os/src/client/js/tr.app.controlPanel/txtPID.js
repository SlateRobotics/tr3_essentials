if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.txtPID = function(id, type) {
  return {
    type: "container",
    size: {
      w: 1 / 18,
      h: 20
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: "txt-" + id + "-" + type,
        type: "text",
        text: "-",
        textSize: 14,
        onDraw: function () {
          if (tr.data.joints[id].pid) {
            if (type == "P") {
              this.text = tr.data.joints[id].pid[0].toFixed(1);
            } else if (type == "I") {
              this.text = tr.data.joints[id].pid[1].toFixed(1);
            } else if (type == "D") {
              this.text = tr.data.joints[id].pid[2].toFixed(1);
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
