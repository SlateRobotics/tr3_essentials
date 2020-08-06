if (!tr) tr = {};
if (!tr.app) tr.app = {};

tr.app.settings = new App({
  id: 3,
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
      margin: 10,
      padding: 10,
      background: "rgba(255, 255, 255, 0.2)",
      radius: 15,
      children: [],
    }, {
      type: "container",
      size: {
        w: 0.75,
        h: 1.0
      },
      margin: 10,
      padding: 10,
      background: "rgba(255, 255, 255, 0.2)",
      radius: 15,
      children: [],
    }],
  }],
});
