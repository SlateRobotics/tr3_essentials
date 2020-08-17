if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.tabInverse = function() {
  var c = tr.controls.pnp2;

  var children = [];
  children.push(c.inverseColL())
  children.push(c.inverseColM())
  children.push(c.inverseColR())

  return {
    type: "container",
    border: false,
    size: {
      w: 1,
      h: 1,
    },
    children: children,
  }
}
