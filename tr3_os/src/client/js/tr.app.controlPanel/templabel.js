function templabel(text, w, h) {
  var ww = 0;
  var hh = 0;
  if (w == 0) {
    ww = 1;
  } else {
    ww = w;
  }
  if (h == 0) {
    hh = 25;
  } else {
    hh = h;
  }
  return {
    type: "container",
    size: {
      w: ww,
      h: hh
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: text,
        textSize: 18,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
