if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.txtD = function(id) {
  return {
    type: "container",
    size: {
      w: 1/18,
      h: 20
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: "txt-" + id + "-d",
        type: "text",
        text: "0.2",
        textSize: 14,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
