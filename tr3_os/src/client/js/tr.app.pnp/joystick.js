if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.joystick = function() {
  return {
    type: "container",
    size: {
      w: 100,
      h: 100
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "joystick",
        size: {
          w: 100,
          h: 100
        }
      }],
    }]
  }
}
