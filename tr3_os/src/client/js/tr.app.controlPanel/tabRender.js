if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.tabRender = function() {
  var c = tr.controls.controlPanel;

  return {
    type: "container",
    size: {
      w: 1.0,
      h: "fill"
    },
    padding: 10,
    background: "rgb(100, 100, 100)",
    children: [{
      type: "container",
      border: false,
      size: {
        w: 1.0,
        h: "fill"
      },
      padding: 0,
      children: [{
        type: "tr2",
      }]
    }],
  }
}
