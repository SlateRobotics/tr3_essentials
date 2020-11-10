tr.controls.controlPanel.controlVoltage = function() {
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
        text: "Voltage",
        size: { w: 0.5, h: 70 },
        textSize: 20,
        padding: 15,
        align: {
          v: "CENTER",
          h: "LEFT"
        },
      }, {
        type: "text",
        text: "0.00 V",
        size: { w: 0.5, h: 70 },
        textSize: 18,
        padding: 15,
        align: {
          v: "CENTER",
          h: "RIGHT"
        },
        onDraw: function () {
          var app = this.getApp();
          var page = app.getCurrentPage();
          var id = tr.controlPanel.state.currentActuator;
          var slider = page.getChild("slider-control-voltage").element;
          this.text = (slider.value() * 12.6).toFixed(2) + " V";
        },
      }, c.slider("voltage")],
    }]
  }
}
