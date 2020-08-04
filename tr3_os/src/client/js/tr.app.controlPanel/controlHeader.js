if (!tr) tr = {};
if (!tr.app) tr.app = {};
if (!tr.app.controlPanel) tr.app.controlPanel = {};

tr.app.controlPanel.controlHeader = function() {
  var c = tr.app.controlPanel;
  return [c.label("IDs"), c.label("Position"), c.label("Mode Select", .333), c.label("Target"), c.label("Position Slider", .333)];
}
