tr.controls.frv.tabHead = function() {
  var c = tr.controls.frv;

  return {
    type: "container",
    size: {
      w: 1.0,
      h: "fill"
    },
    background: "rgb(50, 50, 50)",
    children: [
      c.btnHead("◤"), c.btnHead("▲"), c.btnHead("◥"),
      c.btnHead("◀"), c.btnHead("●"), c.btnHead("▶"),
      c.btnHead("◣"), c.btnHead("▼"), c.btnHead("◢"),
    ],
  }
}
