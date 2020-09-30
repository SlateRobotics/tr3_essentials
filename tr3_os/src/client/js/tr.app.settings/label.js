tr.controls.settings.label = function(lbl, b) {
  if (!b) b = 115;
  return {
    type: "container",
    background: b,
    size: {
      w: 1,
      h: 35
    },
    padding: 10,
    margin: 0,
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: lbl,
        textSize: 16,
        align: {
          v: "CENTER",
          h: "LEFT"
        },
      }],
    }]
  }
}
