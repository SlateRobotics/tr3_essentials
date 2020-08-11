tr.app.frv = function() {
  var c = tr.controls.frv;

  return new App({
    id: 3,
    name: "Teleop",
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
        text: "Teleoperation",
      },
      children: [{
        type: "container",
        size: {
          w: 0.75,
          h: 1.0,
        },
        padding: 0,
        margin: 0,
        background: "rgb(34, 34, 34)",
        children: [{
          type: "tabControl",
          padding: 0,
          margin: 0,
          size: {
            w: 1.0,
            h: "fill",
          },
          labels: ["First-Person", "Third-Person"],
          pages: [{
            type: "container",
            size: {
              w: 1.0,
              h: "fill",
            },
            background: "rgb(34, 34, 34)",
            children: [{
              type: "camera"
            }]
          }, {
            type: "container",
            size: {
              w: 1.0,
              h: "fill",
            },
            background: "rgb(100, 100, 100)",
            children: [{
              type: "tr2"
            }]
          }],
        }],
      }, {
        type: "container",
        size: {
          w: 0.25,
          h: 1.0,
        },
        padding: 0,
        margin: 0,
        background: "rgb(34, 34, 34)",
        children: [{
          type: "container",
          size: {
            w: 1.0,
            h: 0.4,
          },
          background: "rgb(34, 34, 34)",
          children: [{
            type: "minimap",
            size: {
              w: 1,
              h: 1
            }
          }]
        }, {
          type: "container",
          size: {
            w: 1.0,
            h: 0.6,
          },
          padding: 0,
          margin: 0,
          background: "rgb(34, 34, 34)",
          children: [{
            type: "tabControl",
            padding: 0,
            margin: 0,
            labels: ["Base", "Arm", "Head"],
            pages: [c.tabBase(), c.tabArm(), c.tabHead()],
          }]
        }]
      }],
    }],
  });
};
