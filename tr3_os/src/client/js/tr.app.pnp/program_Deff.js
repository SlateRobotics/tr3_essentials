if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};
var p = tr.controls.pnp2.program_Tools;

tr.controls.pnp2.waypoint = function(config) {
  this.config = config;
  if (!this.config) this.config = {};
  this.positions = this.config.positions || [0, 0, 0, 0, 0];
  this.speed = this.config.speed || 1;

  this.pose = config.pose || {
    position: { x: 0, y: 0, z: 0 },
    orientation: { x: 0, y: 0, z: 0, w: 0 }
  }

  this.config.positions = this.positions;
  this.config.speed = this.speed;

  this.setup = function () {
    this.setPose();
  }

  this.incrementPosition = function(idx, i) {
    this.positions[idx] += i;
  }

  this.setwpPositions = function(pos) {
    this.positions = pos;
  }

  this.incrementDuration = function(i) {
    this.speed += i;
    if (this.speed < 0) {
      this.speed = 0;
    }
  }

  this.clone = function () {
    return new tr.controls.pnp2.waypoint({
      positions: this.positions,
      speed: this.speed,
      pose: {
        position: this.pose.position,
        orientation: this.pose.orientation,
      }
    });
  }

  this.setPose = function () {
    var state = {
      "a0": this.positions[0],
      "a1": this.positions[1],
      "a2": this.positions[2],
      "a3": this.positions[3],
      "a4": this.positions[4],
    }

    tr.data.getForwardIk(state, function (pose) {
      tr.controls.pnp2.desiredPose = pose;
      this.pose = pose;
    }.bind(this));
  }

  this.setup();
}

tr.controls.pnp2.program = function(config) {
  this.config = config || {};
  this.id = config.id || 0;
  this.name = config.name || "Program " + this.id;
  this.waypoints = [];
  this.currentWaypoint = -1;

  this.setup = function() {
    this.config.waypoints = this.config.waypoints || [];
    for (var i = 0; i < this.config.waypoints.length; i++) {
      var wp = this.config.waypoints[i];
      if (wp.config) {
        this.waypoints.push(wp);
      } else {
        this.waypoints.push(new tr.controls.pnp2.waypoint(wp));
      }
    }

    if (this.waypoints.length > 0) {
      this.currentWaypoint = 0;
    }

    this.open();
  }

  this.setPositions = function(pos, app) {
    if (this.waypoints[this.currentWaypoint]) {
      this.waypoints[this.currentWaypoint].setwpPositions(pos);
      p.updateUI(app)
    }

    this.save();
  }

  this.getCurrentWaypoint = function() {
    return this.waypoints[this.currentWaypoint];
  }

  this.insertWaypoint = function(app) {
    /*var wp = [];
    //console.log(this);
    for (var i = 0; i < this.waypoints.length; i++) {
      wp.push(this.waypoints[i]);
      if (i == this.currentWaypoint) {
        var config = Object.assign({}, this.waypoints[i].config);
        config.positions = Object.assign([], config.positions);
        wp.push(new tr.controls.pnp2.waypoint(config));
      }
    }

    if (wp.length == 0) {
      wp.push(new tr.app.pnp.waypoint());
      this.currentWaypoint = 0;
    } else {
      this.currentWaypoint += 1;
    }*/

    this.waypoints.splice(this.currentWaypoint + 1, 0, this.waypoints[this.currentWaypoint].clone());
    this.currentWaypoint += 1;

    p.updateUI(app)
  }

  this.removeWaypoint = function(app) {
    var wp = [];
    for (var i = 0; i < this.waypoints.length; i++) {
      if (i != this.currentWaypoint) {
        wp.push(this.waypoints[i]);
      }
    }
    this.waypoints = wp;

    if (this.currentWaypoint >= this.waypoints.length) {
      this.currentWaypoint = this.waypoints.length - 1;
    }

    if (this.waypoints.length == 0) {
      this.insertWaypoint();
    }
    p.updateUI(app)
  }

  this.incrementWaypoint = function(i, app) {
    this.currentWaypoint += i;
    if (this.currentWaypoint < 0) {
      this.currentWaypoint = 0;
    } else if (this.currentWaypoint >= this.waypoints.length) {
      this.currentWaypoint = this.waypoints.length - 1;
    }
    p.updateUI(app)

    this.waypoints[this.currentWaypoint].setPose();
  }

  this.open = function () {
    tr.data.readFile({
      app: "tr.app.pnp",
      fileName: "program-" + this.id + ".json",
      callback: function (data) {
        try {
          var wp = JSON.parse(data.contents).waypoints;
          this.waypoints = [];
          for (var i = 0; i < wp.length; i++) {
            this.waypoints.push(new tr.controls.pnp2.waypoint(wp[i]))
          }
        } catch (err) {

        }
      }.bind(this)
    });
  }

  this.delete = function () {
    tr.data.deleteFile({
      app: "tr.app.pnp",
      fileName: "program-" + this.id + ".json",
    });
  }

  this.save = function () {
    if (this.waypoints.length == 0) {
      return;
    }

    var f = {
      id: this.id,
      name: this.name,
      waypoints: [],
    }

    for (var i = 0; i < this.waypoints.length; i++) {
      f.waypoints.push({
        positions: this.waypoints[i].positions,
        speed: this.waypoints[i].speed,
        pose: this.waypoints[i].pose
      });
    }

    tr.data.writeFile({
      app: "tr.app.pnp",
      fileName: "program-" + f.id + ".json",
      contents: JSON.stringify(f),
    });
  }

  this.setup();
}
