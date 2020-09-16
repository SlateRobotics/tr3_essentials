if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.waypointDisplay = function() {
  return {
    type: "container",
    size: {
      w: 0.75/10,
      h: 1.0
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: "dwaypoint",
        type: "text",
        text: "0", // Current Waypoint ID
        textSize: 24,
        textFont: "roboto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
