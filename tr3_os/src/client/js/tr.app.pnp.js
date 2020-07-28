app.pnp = new App({
  name: "Pick N Place",
  desc: "Pick and Place",
  iconUrl: "/img/icon-app-pnp",

  programs: [],
  currentProgram: -1,
  waypointStart: 0,
  programMode: 0, // 0 = edit, 1, playback
  robotState: [],
	  
  setup: function() {
    var app = this._app;
    app.addPages(tr.app.pnp.pages);

    var page = app.pages[app.pageCurrent];
    page.onDraw = function() {
      this.app.config.programRun();
    };

    this.currentProgram = 0;
    this.programs.push(new tr.app.pnp.program({
      id: 0,
      name: "Program 0",
      waypoints: [{
        positions: [0, 0, 10, 0, 0],
        speed: 1
      }, {
        positions: [20, 20, 20, 20, 20],
        speed: 3
      }, {
        positions: [70, 70, 70, 70, 70],
        speed: 5
      }]
    }));

    this.programs.push(new tr.app.pnp.program({
      id: 1,
      name: "Program 1",
      waypoints: [{
        positions: [0, 0, 20, 0, 0],
        speed: 3
      }, {
        positions: [80, 80, 80, 80, 80],
        speed: 6
      }, {
        positions: [90, 40, 30, 60, 10],
        speed: 2
      }]
    }));

    this.robotState = this.getCurrentProgram().getCurrentWaypoint().positions;
    this.updateUI();
  },

  addProgram: function() {
    var id = 0;
    for (var i = 0; i < this.programs.length; i++) {
      if (this.programs[i].id >= id) {
        id = this.programs[i].id + 1;
      }
    }

    this.programs.push(new tr.app.pnp.program({
      id: id
    }));

    this.updateUI();
  },

  changeProgram: function(name) {
    for (var i = 0; i < this.programs.length; i++) {
      if (this.programs[i].name == name) {
        this.currentProgram = i;
        this.updateUI();
      }
    }
  },

  getCurrentProgram: function() {
    return this.programs[this.currentProgram];
  },

  programStart: function() {
    var prog = this.getCurrentProgram();
    prog.currentWaypoint = 0;
    var wp = prog.getCurrentWaypoint().positions;
    this.waypointStartPos = Object.assign([], wp);
    if (prog.waypoints.length > 2) {
      prog.currentWaypoint += 1;
      this.programMode = 1;
    }
  },

  programStartFrom: function() {
    var prog = this.getCurrentProgram();
    var wp = prog.getCurrentWaypoint().positions;
    this.waypointStartPos = Object.assign([], wp);
    if (prog.currentWaypoint < prog.waypoints.length - 1) {
      prog.currentWaypoint += 1;
      this.programMode = 1
    } else {
      this.programMode = 0;
    }
  },

  programStop: function() {
    this.programMode = 0;
  },

  programRun: function() {
    if (this.programMode == 1) {
      var prog = this.getCurrentProgram();
      var wp = prog.getCurrentWaypoint();
      var pos = wp.positions;
      var wpDuration = wp.speed; // seconds

      var startPos = this.waypointStartPos;

      var time = new Date();
      var duration = (time - this.waypointStart) / 1000.0;
      var durationComplete = (duration / wpDuration)

      if (wpDuration == 0) {
        durationComplete = 1.0;
      }

      for (var i = 0; i < this.robotState.length; i++) {
        this.robotState[i] = (pos[i] - startPos[i]) * durationComplete + startPos[i];
      }

      this.updateUI();

      if (duration >= wpDuration) {
        if (prog.waypoints.length - 1 <= prog.currentWaypoint) {
          this.programMode = 0;
        } else {
          this.waypointStart = new Date();
          this.waypointStartPos = Object.assign([], pos);
          prog.currentWaypoint += 1;
        }
      }
    } else {
      var pos = this.getCurrentProgram().getCurrentWaypoint().positions;
      this.waypointStart = new Date();
      this.waypointStartPos = Object.assign([], pos);
      this.robotState = Object.assign([], pos);
      this.updateUI();
    }
  },

  updateUI: function() {
    var prog = this.getCurrentProgram();
    var positions = prog.getCurrentWaypoint().positions;
    var speed = prog.getCurrentWaypoint().speed;

    var page = this._app.getCurrentPage();
    var tr2 = page.getChild('tr').tr2;
    tr2.state.a0 = this.robotState[0];
    tr2.state.a1 = this.robotState[1];
    tr2.state.a2 = this.robotState[2];
    tr2.state.a3 = this.robotState[3];
    tr2.state.a4 = this.robotState[4];

    page.getChild('dwaypoint').text = prog.currentWaypoint;
    // page.getChild('dspeed').text = Math.round(speed * 100) / 100;
    // page.getChild('d0').text = Math.floor(positions[0]) + "°";
    // page.getChild('d1').text = Math.floor(positions[1]) + "°";
    // page.getChild('d2').text = Math.floor(positions[2]) + "°";
    // page.getChild('d3').text = Math.floor(positions[3]) + "°";
    // page.getChild('d4').text = Math.floor(positions[4]) + "°";

    this.updateSelect();
  },

  updateSelect: function() {
    var page = this._app.pages[this._app.pageCurrent];
    var sel = page.getChild('progselect');
    var options = sel.options;

    var opts = [];
    var update = false;
    for (i = 0; i < this.programs.length; i++) {
      opts.push(this.programs[i].name);

      if (options.length - 1 < i) {
        update = true;
      } else if (opts[i] != options[i]) {
        update = true;
      }
    }

    if (update) {
      page.getChild('progselect').setOptions(opts);
    }
  }
});
