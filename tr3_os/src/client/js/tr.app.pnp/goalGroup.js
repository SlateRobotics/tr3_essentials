if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.goalGroup = function() {
  var c = tr.controls.pnp2;

  var children = [];
  children.push(c.goalHeader_Label("Target"));
  children.push(c.goalHeader_Label("Current"));
  children.push(c.goalID_Label("a0"));
  children.push(c.goalTarget_Label("a0"));
  children.push(c.goalID_Label("a0"));
  children.push(c.goalCurrent_Label("a0"));

  children.push(c.goalID_Label("a1"));
  children.push(c.goalTarget_Label("a1"));
  children.push(c.goalID_Label("a1"));
  children.push(c.goalCurrent_Label("a1"));

  children.push(c.goalID_Label("a2"));
  children.push(c.goalTarget_Label("a2"));
  children.push(c.goalID_Label("a2"));
  children.push(c.goalCurrent_Label("a2"));

  children.push(c.goalID_Label("a3"));
  children.push(c.goalTarget_Label("a3"));
  children.push(c.goalID_Label("a3"));
  children.push(c.goalCurrent_Label("a3"));

  children.push(c.goalID_Label("a4"));
  children.push(c.goalTarget_Label("a4"));
  children.push(c.goalID_Label("a4"));
  children.push(c.goalCurrent_Label("a4"));

  children.push(c.goalID_Label("Duration Total"));
  children.push(c.goalDurTotal_Label());
  children.push(c.goalID_Label("Duration Passed"));
  children.push(c.goalDurCompleted_Label());
  return {
    type: "container",
    size: {
      w: 1,
      h: 0.288,
    },
    background: "rgba(255, 255, 255, 0.2)",
    margin: 2,
    padding: 5,
    children: [{
      type: "container",
      border: false,
      children: children,
    }]
  }
}
