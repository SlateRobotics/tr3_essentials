if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.configRow = function(id) {
  var c = tr.controls.controlPanel;
  var children = [];
  children.push(c.labelID(id, 3/18));
  children.push(c.btnMotorDir(id));
  children.push(c.btnResetPos(id));
  children.push(c.btnCalibrate(id));
  children.push(c.btnPID(id, "P", "◀"))
  children.push(c.txtPID(id, "P"));
  children.push(c.btnPID(id, "P", "▶"))
  children.push(c.btnPID(id, "I", "◀"))
  children.push(c.txtPID(id, "I"));
  children.push(c.btnPID(id, "I", "▶"))
  children.push(c.btnPID(id, "D", "◀"))
  children.push(c.txtPID(id, "D"));
  children.push(c.btnPID(id, "D", "▶"))
  return children;
}
