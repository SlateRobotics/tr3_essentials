if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.sliderRow = function(id) {
  var c = tr.controls.pnp2;

  var children = [];

  children.push(c.sliderID_Label(id));
  children.push(c.sliderValue_Label(id));
  children.push(c.slider(id));

  return {
    type: "container",
    border: false,
    size: {
      w: 1,
      h: 20,
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
