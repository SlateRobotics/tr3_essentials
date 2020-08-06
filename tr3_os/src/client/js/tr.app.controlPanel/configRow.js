if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.configRow = function(id) {
  var c = tr.controls.controlPanel;
  return [c.labelID(id), c.btnMotorDir(id), c.btnResetPos(id), c.btnCalibrate(id), c.txtP(id), c.sliderP(id), c.txtI(id), c.sliderI(id), c.txtD(id), c.sliderD(id), c.btnPID(id)];
}
