if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.tabControl = function() {
  var c = tr.controls.controlPanel;

  var children = [];
  //children.push(c.selectMode());
  children.push(c.controlPosition());
  children.push(c.controlVelocity());
  children.push(c.controlTorque());
  children.push(c.controlVoltage());

  return {
    type: "container",
    size: {
      w: 1.0,
      h: "fill"
    },
    padding: 10,
    background: "rgba(255, 255, 255, 0.2)",
    children: [
      c.selectMode(), {
        type: "container",
        size: {
          w: 1.0,
          h: 0.85
        },
        padding: 0,
        margin: 0,
        children: children,
      }]
  }
}
