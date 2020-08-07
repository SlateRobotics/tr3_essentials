if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.playbackLabel = function(id) {
  return {
    type: "container",
    size: {
      w: 1 / 4,
      h: 1 / 4
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: id,
        textSize: 14,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
