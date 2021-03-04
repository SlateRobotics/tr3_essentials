tr.controls.frv.tabArm = function() {
  var c = tr.controls.frv;

  return {
    type: "container",
    size: {
      w: 1.0,
      h: "fill"
    },
    background: "rgb(80, 80, 80)",
    children: [
      c.btnArm("◤"), c.btnArm("▲"), c.btnArm("◥"),
      c.btnArm("◀"), c.btnArm("●"), c.btnArm("▶"),
      c.btnArm("◣"), c.btnArm("▼"), c.btnArm("◢"),
      c.btnArm("Z+"), c.btnArm("G+"), c.btnArm(""),
      c.btnArm("Z-"), c.btnArm("G-"), c.btnArm(""),
    ],
  }
}
