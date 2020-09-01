if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.btnToglStop = function() {
  return {
    type: "container",
    size: {
      w: 3 / 9,
      h: 120
    },
    background: "red",
    onSetup: function () {
      this.mode = "STOPPED";
    },
    onDraw: function () {
      var app = this.getApp();
      var page = app.getCurrentPage();

      if (this.mode == "STOPPED") {
        this.background = "red"
        page.getChild("ToglStop").text = "STOP";
      } else {
        this.background = "green";
        page.getChild("ToglStop").text = "RELEASE";
      }
    },
    onClick: function() {
      if (this.mode == "STOPPED") {
        this.mode = "RELEASED"
        tr.data.socket.emit("/tr3/stop", true, );
      } else if (this.mode == "RELEASED") {
        this.mode = "STOPPED"
        tr.data.socket.emit("/tr3/stop", false, );
      }

    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: "ToglStop",
        type: "text",
        text: "STOP",
        textSize: 52,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
