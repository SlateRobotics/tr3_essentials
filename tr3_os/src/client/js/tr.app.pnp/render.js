if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.render = function() {
  return {
    type: "container",
    size: {
      w: 0.75,
      h: "fill",
    },
    children: [{
      id: 'tr',
      type: "tr2",
      useLiveState: false,
    }],
  }
}
