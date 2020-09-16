if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.controls = function() {
  var c = tr.controls.pnp2;

  var children = [];
  children.push(c.tabControl());

  return {
    type: "container",
    size: {
      w: 0.25,
      h: "fill",
    },
    border: true,
    children: [{
      type: "container",
      children: children,
    }]
  }
}
