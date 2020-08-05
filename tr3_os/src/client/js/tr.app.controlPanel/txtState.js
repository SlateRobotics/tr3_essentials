if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.txtState = function(id) {
  return {
    type: "container",
    size: {
      w: 0.111,
      h: 20
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: "",
        textSize: 14,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
        onDraw: function() {
          var p = tr.data.getState(id).position;
          if (p) {
            this.text = p.toFixed(2);
          }
        }
      }],
    }]
  }
}
