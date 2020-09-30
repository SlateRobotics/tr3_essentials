if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.label = function(id, w, h) {
  if (!w) w = 1.0 / 9.0;
  if (!h) h = 35;
  return {
    type: "container",
    size: {
      w: w,
      h: h
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: id,
        textSize: 16,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
