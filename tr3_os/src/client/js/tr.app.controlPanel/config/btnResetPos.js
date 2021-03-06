if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.btnResetPos = function() {
  return {
    type: "container",
    background: "rgb(75,75,75)",
    size: {
      w: 1/3,
      h: 75
    },
    margin: 5,
    onClick: function() {
      var id = tr.controlPanel.state.currentActuator;
      tr.data.socket.emit("/tr3/" + id + "/reset/position", true);
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: "RESET POSITION",
        textSize: 18,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
