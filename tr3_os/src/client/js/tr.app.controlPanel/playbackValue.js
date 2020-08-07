if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.playbackValue = function(id) {
  return {
    type: "container",
    size: {
      w: 1 / 4,
      h: 1 / 4
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: "0.00",
        textSize: 14,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
        onDraw: function() {
          // Replace with Waypoint Controll Stuff
          var p = tr.data.getState(id).position;
          if (p) {
            this.text = p.toFixed(2);
          }
        }
      }],
    }]
  }
}
