if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.configHeader = function() {
  var c = tr.controls.controlPanel;
  return [c.label("IDs", 3/18), c.label("Motor Dir"), c.label("Reset Pos"), c.label("Calibrate"), c.label("PID Tuning", 9/18)];
}
