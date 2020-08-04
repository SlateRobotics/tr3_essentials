if (!tr) tr = {};
if (!tr.app) tr.app = {};
if (!tr.app.controlPanel) tr.app.controlPanel = {};

tr.app.controlPanel.tabRender = function() {
  var c = tr.app.controlPanel;

  return {
    type: "container",
    size: {
      w: 1.0,
      h: "fill"
    },
    padding: 10,
    background: "rgba(255, 255, 255, 0.2)",
    children: [{
      type: "tr2",
    }],
  }
}
