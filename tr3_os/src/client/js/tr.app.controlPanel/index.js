tr.app.controlPanel = function() {
  var c = tr.controls.controlPanel;

  return new App({
    id: 1,
    name: "Control Panel",
    iconUrl: "/img/icon-app-control",
    pages: [{
      header: {
        text: "Control Panel",
      },
      children: [{
        type: "container",
        children: [{
          type: "listBox",
          size: { w: 0.2, h: 1.0 },
          items: ["a0", "a1", "a2", "a3", "a4", "g0", "h0", "h1", "b0", "b1"],
          onChange: function (item) {
            tr.controlPanel.state.currentActuator = item;
          },
        }, {
          type: "tabControl",
          size: { w: 0.7999, h: 1.0 },
          labels: ["Control", "Config"],
          pages: [c.tabControl(), c.tabConfig()],
        }]
      }],
    }],
  });
};
