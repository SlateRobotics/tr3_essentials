if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.programHeader = function() {
  var c = tr.controls.pnp2;

  var children = [];

  children.push(c.programSelect());
  children.push(c.programBtn_Delete());
  children.push(c.programBtn_Add());
  children.push(c.programBtn_Settings());

  children.push(c.waypointBtn_Previous());
  children.push(c.waypointDisplay());
  children.push(c.waypointBtn_Next());
  children.push(c.waypointBtn_Delete());
  children.push(c.waypointBtn_Add());
  return {
    type: "container",
    border: false,
    size: {
      w: 1,
      h: 100,
    },
    children: children,
  }
}
