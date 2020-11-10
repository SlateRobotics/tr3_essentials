tr.controls.controlPanel.tabConfig = function() {
  var c = tr.controls.controlPanel;

  var children = [];
  children.push(c.btnMotorDir());
  children.push(c.btnResetPos());
  children.push(c.btnCalibrate());
  children.push(c.gains("position"));
  children.push(c.gains("velocity"));
  children.push(c.gains("torque"));

  return {
    type: "container",
    size: {
      w: 1.0,
      h: "fill"
    },
    padding: 10,
    background: "rgba(255, 255, 255, 0.2)",
    children: children,
  }
}
