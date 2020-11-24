tr.controls.controlPanel.controlVelocity = function() {
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
        text: "Velocity",
        size: { w: 0.5, h: 50 },
        textSize: 18,
        padding: 15,
        align: {
          v: "CENTER",
          h: "LEFT"
        },
      }, {
        type: "text",
        text: "0.00 rad/sec",
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
          var vel = state.velocity || 0;
          this.text = vel.toFixed(2) + " rad/sec";
        },
      }, c.slider("velocity")],
    }],
    onDraw: function () {
      this.size.h = this.parent.size.h / this.parent.children.length;
    }
  }
}
