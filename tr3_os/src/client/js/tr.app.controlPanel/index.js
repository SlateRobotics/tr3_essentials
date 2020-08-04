app.drawing = new App({
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
      pages: [{
        type: "container",
        size: {
          w: 1.0,
          h: "fill"
        },
        padding: 10,
        background: "rgba(255, 255, 255, 0.2)",
        children: [
          label("IDs"), label("Position"), label("Mode Select", .333), label("Target"), label("Position Slider", .333),
          label("a0"), label_state("a0"), select_mode("a0"), slider_label("a0"), pos_Slider("a0"),
          label("a1"), label_state("a1"), select_mode("a1"), slider_label("a1"), pos_Slider("a1"),
          label("a2"), label_state("a2"), select_mode("a2"), slider_label("a2"), pos_Slider("a2"),
          label("a3"), label_state("a3"), select_mode("a3"), slider_label("a3"), pos_Slider("a3"),
          label("a4"), label_state("a4"), select_mode("a4"), slider_label("a4"), pos_Slider("a4"),
          label("h0"), label_state("h0"), select_mode("h0"), slider_label("h0"), pos_Slider("h0"),
          label("h1"), label_state("h1"), select_mode("h1"), slider_label("h1"), pos_Slider("h1"),
          {
            type: "container",
            size: {
              w: 1.0,
              h: 10
            },
            border: false
          },
          button_big("STOP", "/tr3/stop", true, "rgba(212, 40, 40, 1)"), button_big("RELEASE", "/tr3/stop", false, "rgba(43, 212, 40, 1)"),
          {
            type: "container",
            size: {
              w: 1.0,
              h: 10
            },
            border: false
          },
          button_big("SHUTDOWN", "/tr3/shutdown", true, "rgba(212, 40, 40, 1)"), button_big("POWER UP", "/tr3/powerup", true, "rgba(43, 212, 40, 1)")
        ],
      }, {
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
