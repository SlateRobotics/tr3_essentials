tr.controls.controlPanel.controlPosition = function() {
  var s = tr.controlPanel.state;
  var c = tr.controls.controlPanel;

  return {
    type: "container",
    size: {
      w: 1.0,
      h: 115
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: "Position",
        size: { w: 0.5, h: 70 },
        textSize: 20,
        padding: 15,
        align: {
          v: "CENTER",
          h: "LEFT"
        },
      }, {
        type: "text",
        text: "0.00 rad",
        size: { w: 0.5, h: 70 },
        textSize: 18,
        padding: 15,
        align: {
          v: "CENTER",
          h: "RIGHT"
        },
        onDraw: function () {
          var id = tr.controlPanel.state.currentActuator;
          var state = tr.data.getState(id);
          var pos = state.position || 0;
          this.text = pos.toFixed(2) + " rad";
        },
      }, c.slider("position")],
    }]
  }
}
