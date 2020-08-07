if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.btnPlaybackImg = function(url, rostopic, value) {
  return {
    type: "container",
    size: {
      w: 1 / 4,
      h: 50
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        if (rostopic && value) {
          tr.data.socket.emit(rostopic, value);
        }
      },
      children: [{
        type: "image",
        url: url,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
