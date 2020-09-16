if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.body = function() {
  var c = tr.controls.pnp2;

  var children = [];
  children.push(c.render());
  children.push(c.controls());

  return {
    type: "container",
    size: {
      w: 1.0,
      h: "fill",
    },
    background: "rgba(255, 255, 255, 0.2)",
    margin: 0,
    padding: 0,
    children: children,
  }
}
