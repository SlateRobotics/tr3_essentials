tr.controls.controlPanel.controlVoltage = function() {
  var s = tr.controlPanel.state;
  var c = tr.controls.controlPanel;

  return {
    type: "container",
    size: {
      w: 1.0,
      h: 90
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: "Voltage",
        size: { w: 0.5, h: 50 },
        textSize: 18,
        padding: 15,
        align: {
          v: "CENTER",
          h: "LEFT"
        },
      }, {
        type: "text",
        text: "0.00 V",
        size: { w: 0.5, h: 50 },
        textSize: 18,
        padding: 15,
        align: {
          v: "CENTER",
          h: "RIGHT"
        },
        onDraw: function () {
          var id = tr.controlPanel.state.currentActuator;
          var state = tr.data.getState(id);
          var vol = state.effort || 0;
          vol = vol / 100.0 * 12.6;
          this.text = vol.toFixed(2) + " V";
        },
      }, c.slider("voltage")],
    }],
    onDraw: function () {
      this.size.h = this.parent.size.h / this.parent.children.length;
    }
  }
}
