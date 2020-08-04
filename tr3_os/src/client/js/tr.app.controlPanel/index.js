app.controlPanel = function() {
  var c = tr.app.controlPanel;

  return new App({
    name: "Control Panel",
    iconUrl: "/img/icon-app-control",
    pages: [{
      pos: {
        x: 0,
        y: 0
      },
      size: {
        w: 1.0,
        h: 1.0
      },
      header: {
        text: "Control Panel V2",
      },
      children: [{
        type: "tabControl",
        labels: ["Control", "Config", "3D Render"],
        pages: [c.tabControl(), c.tabConfig(), c.tabRender()],
      }],
    }],
  });
};
