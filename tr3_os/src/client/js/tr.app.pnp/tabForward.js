if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.tabForward = function() {
  var c = tr.controls.pnp2;

  var children = [];

    children.push(c.sliderRow("a0"));
    children.push(c.sliderRow("a1"));
    children.push(c.sliderRow("a2"));
    children.push(c.sliderRow("a3"));
    children.push(c.sliderRow("a4"));

  return {
    type: "container",
    border:false,
    size:{
      w: 1,
      h: 120,
    },
    children: children,
    }
  }
