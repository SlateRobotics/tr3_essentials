if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.label = function(id, w) {
  if (!w) w = 0.111;
  return {
    type: "container",
    size: {
      w: w,
      h: 20
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
