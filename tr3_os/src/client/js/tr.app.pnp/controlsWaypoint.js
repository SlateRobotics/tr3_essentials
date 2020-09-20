if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.controlsWaypoint = function() {
  var c = tr.controls.pnp2;

  var children = [];
  children.push(c.cell({ label: "Duration", width: 2 }));
  children.push(c.cell({ label: "◀", onClick: function () {
    var app = this.getApp().config;
    var p = tr.controls.pnp2.program_Tools;
    p.getCurrentProgram(app).getCurrentWaypoint(app).speed -= 0.2;
    p.getCurrentProgram(app).save();
  }}));
  children.push(c.cell({ label: "1.0", id: "lbl-wp-duration" }));
  children.push(c.cell({ label: "▶", onClick: function () {
    var app = this.getApp().config;
    var p = tr.controls.pnp2.program_Tools;
    p.getCurrentProgram(app).getCurrentWaypoint(app).speed += 0.2;
    p.getCurrentProgram(app).save();
  }}));

  children.push(c.cell({ label: "Wait", width: 2 }));
  children.push(c.cell({ label: "◀", onClick: function () {
    var app = this.getApp().config;
    var p = tr.controls.pnp2.program_Tools;
    p.getCurrentProgram(app).getCurrentWaypoint(app).wait -= 0.1;
    p.getCurrentProgram(app).save();
  }}));
  children.push(c.cell({ label: "0.0", id: "lbl-wp-wait" }));
  children.push(c.cell({ label: "▶", onClick: function () {
    var app = this.getApp().config;
    var p = tr.controls.pnp2.program_Tools;
    p.getCurrentProgram(app).getCurrentWaypoint(app).wait += 0.1;
    p.getCurrentProgram(app).save();
  }}));

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
