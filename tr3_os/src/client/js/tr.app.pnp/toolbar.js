if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.toolbar = function() {
  var c = tr.controls.pnp2;

  var children = [];
  children.push(c.programSelect());
  children.push(c.programBtn_Add());
  children.push(c.programBtn_Delete());
  children.push(c.programBtn_Play());
  children.push(c.programBtn_Pause());
  children.push(c.waypointBtn_Previous());
  children.push(c.waypointDisplay());
  children.push(c.waypointBtn_Next());
  children.push(c.waypointBtn_Add());
  children.push(c.waypointBtn_Delete());
  children.push(c.programBtn_Send());

  return {
    type: "container",
    size: {
      w: 1.0,
      h: 40,
    },
    background: "rgba(255, 255, 255, 0.2)",
    margin: 0,
    padding: 0,
    children: [{
      type: "container",
      border: false,
      children: children,
    }]
  }
}
