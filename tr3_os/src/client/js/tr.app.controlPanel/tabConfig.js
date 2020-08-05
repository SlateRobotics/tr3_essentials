if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.tabConfig = function() {
  var c = tr.controls.controlPanel;

  return {
    type: "container",
    size: {
      w: 1.0,
      h: "fill"
    },
    padding: 10,
    background: "rgba(255, 255, 255, 0.2)",
  }
}
