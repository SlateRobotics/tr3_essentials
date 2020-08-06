if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.controlRow = function(id) {
  var c = tr.controls.controlPanel;
  return [c.labelID(id), c.txtState(id), c.selectMode(id), c.txtTarget(id), c.slider(id)];
}
