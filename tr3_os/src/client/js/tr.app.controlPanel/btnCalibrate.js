if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.btnCalibrate = function(id) {
  return {
    type: "container",
    background: "rgb(150, 150, 150)",
    size: {
      w: 0.111,
      h: 20
    },
    onClick: function () {
      tr.data.socket.emit("/tr3/joints/" + id + "/calibrate", true);
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: "Calibrate",
        textSize: 14,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
