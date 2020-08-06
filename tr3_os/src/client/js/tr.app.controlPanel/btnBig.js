if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.btnBig = function(text, rostopic, value, background) {
  return {
    type: "container",
    size: {
      w: 0.25,
      h: 50
    },
    background: background,
    onClick: function() {
      tr.data.socket.emit(rostopic, value);
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: text,
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
