if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.btnPID = function(id) {
  return {
    type: "container",
    background: "rgb(150, 150, 150)",
    size: {
      w: 1/18,
      h: 20
    },
    onClick: function () {
      var app = this.getApp();
      var page = app.getCurrentPage();

      var p = page.getChild(id + "slider-p").element.value();
      var i = page.getChild(id + "slider-i").element.value();
      var d = page.getChild(id + "slider-d").element.value();

      tr.data.socket.emit("/tr3/joints/" + id + "/pid", [p, i, d]);
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: "->",
        textSize: 14,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
