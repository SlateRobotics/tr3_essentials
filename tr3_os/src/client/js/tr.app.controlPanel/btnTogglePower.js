if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.btnTogglePower = function() {
  return {
    id: "container-toggle-power",
    type: "container",
    size: {
      w: 3 / 9,
      h: 120
    },
    background: "green",
    onSetup: function () {
      var p0 = tr.data.getState("p0");
      if (p0 && p0.position > 0) {
        this.mode = "OFF";
      } else {
        this.mode = "ON";
      }
    },
    onDraw: function () {
      var app = this.getApp();
      var page = app.getCurrentPage();

      if (this.mode == "ON") {
        this.background = "green";
        page.getChild("btn-toggle-power").text = "POWER ON";
      } else {
        this.background = "red";
        page.getChild("btn-toggle-power").text = "POWER OFF";
      }
    },
    onClick: function() {
      if (this.mode == "ON") {
        this.mode = "OFF";
        tr.data.socket.emit("/tr3/shutdown", true);
      } else {
        this.mode = "ON";
        tr.data.socket.emit("/tr3/powerup", true);
      }

    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: "btn-toggle-power",
        type: "text",
        text: "POWER ON",
        textSize: 42,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
