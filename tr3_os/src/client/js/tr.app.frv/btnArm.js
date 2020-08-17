tr.controls.frv.btnArm = function() {
  var c = tr.controls.frv;

  return {
    type: "container",
    size: {
      w: 0.333,
      h: 50,
    },
    background: "rgba(255, 255, 255, 0.2)",
    children: [{
      type: "text",
      text: "x",
      align: {
        v: "CENTER",
        h: "CENTER"
      },
      size: {
        w: 1,
        h: 1
      },
    }]
  }
}
