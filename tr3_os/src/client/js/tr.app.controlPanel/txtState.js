if (!tr) tr = {};
if (!tr.app) tr.app = {};
if (!tr.app.controlPanel) tr.app.controlPanel = {};

tr.app.controlPanel.txtState = function (id) {
  return {
    type: "container",
    size: {
      w: 0.111,
      h: 25
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: "",
        textSize: 18,
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
