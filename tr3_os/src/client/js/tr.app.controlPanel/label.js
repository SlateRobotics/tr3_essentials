function label(id, w) {
  if (!w) w = 0.111;
  return {
    type: "container",
    size: {
      w: w,
      h: 25
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: id,
        textSize: 18,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
