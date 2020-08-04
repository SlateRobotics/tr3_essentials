app.controlPanel = function () {
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
        pages: [c.tabControl(), {
          type: "container",
          size: {
            w: 1.0,
            h: "fill"
          },
          padding: 10,
          background: "rgba(255, 255, 255, 0.2)",
        }, {
          type: "container",
          size: {
            w: 1.0,
            h: "fill"
          },
          padding: 10,
          background: "rgba(255, 255, 255, 0.2)",
          children: [{
            type: "tr2",
          }],
        }],
      }],
    }],
  });
};
