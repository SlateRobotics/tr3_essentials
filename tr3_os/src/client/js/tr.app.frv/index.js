if (!tr) tr = {};
if (!tr.app) tr.app = {};

tr.app.frv = function() {
  var c = tr.controls.frv;

  return new App({
    id: 3,
    name: "FRV",
    iconUrl: "/img/icon-app-frv",
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
        text: "First-Robot View",
      },
      children: [{
        type: "container",
        size: {
          w: 1.0,
          h: 1.0,
        },
        background: "rgba(255, 255, 255, 0.2)",
        children: [{
          type: "camera"
        }],
      }],
    }],
  });
};
