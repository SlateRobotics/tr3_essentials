tr.app.settings = function () {
  var c = tr.controls.settings;

  return new App({
    id: 4,
    name: "Settings",
    iconUrl: "/img/icon-app-settings",
    pages: [{
      id: "main",
      pos: {
        x: 0,
        y: 0
      },
      size: {
        w: 1.0,
        h: 1.0
      },
      header: {
        text: "Settings",
      },
      children: [{
        type: "container",
        size: {
          w: 0.25,
          h: 1.0
        },
        margin: 0,
        padding: 0,
        background: "rgba(255, 255, 255, 0.2)",
        children: [c.label("POWER", 150)],
      }, {
        type: "container",
        size: {
          w: 0.75,
          h: 1.0
        },
        margin: 0,
        padding: 10,
        background: "rgba(255, 255, 255, 0.2)",
        children: [c.btnTogglePower()],
      }],
    }],
  });
}
