if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.inverseColL = function() {
  var c = tr.controls.pnp2;

  var children = [];
  children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_moveUp());
  children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_zUp());
  children.push(c.inverseBtn_moveLeft());
  children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_moveRight());
  children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_Display("Z"));
  children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_moveDown());
  children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_zDown());

  return {
    type: "container",
    size: {
      w: 1.0,
      h: 1.0,
    },
    margin: 0,
    padding: 0,
    children: [{
      type: "container",
      border: false,
      margin: 0,
      padding: 0,
      children: children,
    }]
  }
}
