if (!tr) tr = {};
if (!tr.app) tr.app = {};

tr.app.pnp2 = function() {
  var c = tr.controls.pnp2;
  var p = c.program_Tools;

var columns = [];
columns.push(c.columnLeft());
columns.push(c.columnRight());


  return new App({

    id: 4,
    name: "P.N.P. V2",
    iconUrl: "/img/icon-app-pnp",
    programs: [],
    currentProgram: -1,
    waypointStart: 0,
    programMode: 0, // 0 = edit, 1, playback
    robotState: [],
    setup: function() {
        p.Program_Setup(this)
    },
    pages: [{
      pos: {
        x: 0,
        y: 0
      },
      onDraw: function() {
        var app = this.getApp().config;
        p.programRun(app);
      },
      header: {
        text: "P.N.P v2",
      },
      children: columns,
  }],
  });


};
