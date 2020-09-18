if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.controlsWaypoint = function() {
  var c = tr.controls.pnp2;

  var children = [];

  return {
    type: "container",
    size: {
      w: 1,
      h: 0.5,
    },
    border: true,
    children: children
  }
}
