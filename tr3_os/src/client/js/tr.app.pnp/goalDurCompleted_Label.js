if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.pnp2.goalDurCompleted_Label = function() {
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
        id: "durcurrent",
        type: "text",
        text: "---" + "%",
        textSize: 12,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
