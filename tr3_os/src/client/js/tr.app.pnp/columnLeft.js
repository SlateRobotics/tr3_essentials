if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.columnLeft = function() {
  var c = tr.controls.pnp2;

  var children = [];

  children.push(c.programHeader());
  children.push(c.btnSimulated());
  children.push(c.btnRealtime());
  //children.push(c.joystick()); //For Testing - Position values arnt as expected, manualy tuned for this spot.
  children.push(c.tabControl());
  return {
    type: "container",
    size: {
      w: 0.5,
      h: 1,
    },
    background: "rgba(255, 255, 255, 0.2)",
    margin: 10,
    padding: 5,
    children: [{
      type: "container",
      border: false,
      children: children,
}]
  }
}
