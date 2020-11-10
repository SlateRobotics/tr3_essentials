if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.lblPID = function(controller, text) {
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
        type: "text",
        text: text,
        textSize: 18,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
