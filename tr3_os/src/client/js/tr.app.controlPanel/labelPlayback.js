if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.labelProgram = function() {
  return {
    type: "container",
    size: {
      w: 1 / 4,
      h: 50
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: "program-label",
        type: "text",
        text: "P#",
        textSize: 28,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
