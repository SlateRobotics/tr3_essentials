if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.controlHeader = function() {
  var c = tr.controls.controlPanel;
  return [c.label("IDs"), c.label("Position"), c.label("Mode"), c.label("Target"), c.label("Position Slider", 5/9)];
}
