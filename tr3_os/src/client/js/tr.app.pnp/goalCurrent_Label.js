if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.pnp2.goalCurrent_Label = function(id) {
  return {
    type: "container",
    size: {
      w: 1 / 4,
      h: 15
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: id + "currentpos",
        type: "text",
        text: "0.00",
        textSize: 12,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
