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
  children.push(c.inverseBtn_Display('h'));
  children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_moveDown());
  children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_zDown());
  return {
    type: "container",
    size: {
      w: 1 / 3,
      h: 0.3,
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
