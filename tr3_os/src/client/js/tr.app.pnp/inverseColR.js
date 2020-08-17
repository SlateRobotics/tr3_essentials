if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.inverseColR = function() {
  var c = tr.controls.pnp2;

  var children = [];
  children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_durationUp());
  children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_Display('d'));
  children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_durationDown());
  children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_Blnk());
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
