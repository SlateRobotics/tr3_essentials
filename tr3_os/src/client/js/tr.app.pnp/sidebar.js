if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.sidebar = function() {
  var c = tr.controls.pnp2;

  var children = [];
  children.push(c.tabControl());
  children.push(c.controlsWaypoint());

  return {
    type: "container",
    border: true,
    size: {
      w: 0.25,
      h: "fill",
    },
    children: children,
  }
}
