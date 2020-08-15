if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.inverseColM = function() {
  var c = tr.controls.pnp2;

  var children = [];
  children.push(c.inverseBtn_tiltR("x"));children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_tiltR("y"));children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_tiltR("z"));
  children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_tiltL("x"));children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_tiltL("y"));children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_tiltL("z"));
  return {
    type: "container",
    size: {
      w: 1/3,
      h: 0.3
      ,
    },
    background: "rgba(255, 255, 255, 0.2)",
    margin: 2,
    padding: 5,
    children: [{
      type: "container",
      border: false,
      children: children,
}]
  }
}
