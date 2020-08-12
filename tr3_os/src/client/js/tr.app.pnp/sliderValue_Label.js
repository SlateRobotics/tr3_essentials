if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.pnp2.sliderValue_Label = function(id) {
  return {
    type: "container",
    size: {
      w: 2 / 9,
      h: 20
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: id + "sliderl",
        type: "text",
        text: "0.00",
        textSize: 14,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
