function label_state (id) {
  return {
    type: "container",
    size: {
      w: 0.111,
      h: 25
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: "",
        textSize: 18,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
        onDraw: function() {
          var p = tr.data.getState(id).position;
          if (p) {
            this.text = p.toFixed(2);
          }
        }
      }],
    }]
  }
}
