if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.render = function() {
  return {
    type: "container",
    border:false,
    size:{
      w: 1,
      h: 0.9,
    },
    children: [{
    type: "tr2",
      }],
    }
  }
