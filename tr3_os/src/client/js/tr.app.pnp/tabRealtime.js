if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.tabRealtime = function() {
  var c = tr.controls.pnp2;

  var children = [];


  return {
    type: "container",
    border:false,
    size:{
      w: 1,
      h: 1,
    },
    children: children,
    }
  }
