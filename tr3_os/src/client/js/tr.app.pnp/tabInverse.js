if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.tabInverse = function() {
  var c = tr.controls.pnp2;

  var children = [];
  children.push(c.inverseColL())

  return {
    type: "container",
    border: false,
    children: children,
  }
}
