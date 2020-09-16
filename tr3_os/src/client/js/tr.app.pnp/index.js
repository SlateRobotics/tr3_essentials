if (!tr) tr = {};
if (!tr.app) tr.app = {};

tr.app.pnp2 = function() {
  var c = tr.controls.pnp2;
  var p = c.program_Tools;

  var columns = [];
  columns.push(c.toolbar());
  columns.push(c.body());

  var state = {
    "a0": 0,
    "a1": 0,
    "a2": 0,
    "a3": 0,
    "a4": 0,
  }

  return new App({

    id: 2,
    name: "Pick-n-Place",
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
