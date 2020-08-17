if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.btnSimulated = function() {
  return {
    type: "container",
    size: {
      w: 0.5,
      h: 30
    },
    background: "rgba(255, 255, 255, 0.1)",
    onClick: function() {
      //tr.data.socket.emit(rostopic, value);
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: "Simulated",
        textSize: 18,
        padding: 12,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
