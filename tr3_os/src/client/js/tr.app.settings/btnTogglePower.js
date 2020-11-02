tr.controls.settings.btnTogglePower = function() {
  return {
    id: "container-toggle-power",
    type: "container",
    size: {
      w: 1,
      h: 125
    },
    margin: 15,
    background: "green",
    onDraw: function () {
      var p0 = tr.data.getState("p0");
      if (p0 && p0.position > 0) {
        this.mode = "ON";
      } else {
        this.mode = "OFF";
      }

      var app = this.getApp();
      var page = app.getCurrentPage();

      if (this.mode == "OFF") {
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
      } else if (this.mode == "OFF") {
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
