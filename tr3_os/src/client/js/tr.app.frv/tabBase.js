tr.controls.frv.tabBase = function() {
  var c = tr.controls.frv;

  return {
    type: "container",
    size: {
      w: 1.0,
      h: "fill"
    },
    background: "rgb(80, 80, 80)",
    children: [
      c.btnBase("◤"), c.btnBase("▲"), c.btnBase("◥"),
      c.btnBase("◀"), c.btnBase("●"), c.btnBase("▶"),
      c.btnBase("◣"), c.btnBase("▼"), c.btnBase("◢"),
    ],
  }
}
