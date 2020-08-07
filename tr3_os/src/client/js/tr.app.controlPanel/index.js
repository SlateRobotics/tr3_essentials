if (!tr) tr = {};
if (!tr.app) tr.app = {};

tr.app.controlPanel = function() {
  var c = tr.controls.controlPanel;

  return new App({
    id: 1,
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
        text: "Control Panel",
      },
      children: [{
        type: "tabControl",
        labels: ["Control", "Config", "Camera", "3D Render"],
        pages: [c.tabControl(), c.tabConfig(), c.tabCamera(), c.tabRender()],
      }],
    }],
  });
};
