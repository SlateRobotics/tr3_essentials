if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.btnToglStop = function() {
  var currentmode = "STOPPED"
  return {
    type: "container",
    size: {
      w: 3 / 9,
      h: 120
    },
    background: "red",
    onClick: function() {
      if (currentmode == "STOPPED") {
        currentmode = "RELEASED"
        this.background = "green"
        var app = this.getApp();
        var page = app.getCurrentPage();
        page.getChild("ToglStop").text = "RELEASE";
        tr.data.socket.emit("/tr3/stop", true, );
      } else if (currentmode == "RELEASED") {
        currentmode = "STOPPED"
        this.background = "red"
        var app = this.getApp();
        var page = app.getCurrentPage();
        page.getChild("ToglStop").text = "STOP";
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
