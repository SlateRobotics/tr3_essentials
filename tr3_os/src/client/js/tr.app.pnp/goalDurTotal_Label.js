if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.pnp2.goalDurTotal_Label = function() {
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
        id: "durtotal",
        type: "text",
        text: "0.00" + " -Seconds",
        textSize: 12,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
