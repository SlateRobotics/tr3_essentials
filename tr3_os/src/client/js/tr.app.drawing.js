app.drawing = new App({
  name: "Drawing",
  iconUrl: "/img/icon-app-drawing",
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
      text: "Drawing",
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
      children: [{
        type: "tr2",
        onDraw: function() {
          this.tr2.state.a0 += 1;
        }
      }],
    }],
  }],
});
