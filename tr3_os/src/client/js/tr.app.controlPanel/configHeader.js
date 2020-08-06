if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.configHeader = function() {
  var c = tr.controls.controlPanel;
  return [c.label("IDs"), c.label("Motor Dir"), c.label("Reset Pos"), c.label("Calibrate"), c.label("PID Tuning", 5/9)];
}
