if (!tr) tr = {};
if (!tr.app) tr.app = {};
if (!tr.app.controlPanel) tr.app.controlPanel = {};

tr.app.controlPanel.txtTarget = function(id) {
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
        id: id + "sliderl",
        type: "text",
        text: "0.00",
        textSize: 18,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
