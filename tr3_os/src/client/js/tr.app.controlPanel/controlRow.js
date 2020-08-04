if (!tr) tr = {};
if (!tr.app) tr.app = {};
if (!tr.app.controlPanel) tr.app.controlPanel = {};

tr.app.controlPanel.controlRow = function(id) {
  var c = tr.app.controlPanel;
  return [c.label(id), c.txtState(id), c.selectMode(id), c.txtTarget(id), c.slider(id)];
}
