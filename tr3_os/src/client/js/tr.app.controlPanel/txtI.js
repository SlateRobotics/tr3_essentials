if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.txtI = function(id) {
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
        id: "txt-" + id + "-i",
        type: "text",
        text: "5.0",
        textSize: 14,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
