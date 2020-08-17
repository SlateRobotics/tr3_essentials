if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.waypoint = function(config) {
  this.config = config;
  if (!this.config) this.config = {};
  this.positions = this.config.positions || [0, 0, 0, 0, 0];
  this.speed = this.config.speed || 1;

  this.config.positions = this.positions;
  this.config.speed = this.speed;

  this.incrementPosition = function(idx, i) {
    this.positions[idx] += i;
  }

  this.incrementDuration = function(i) {
    this.speed += i;
    if (this.speed < 0) {
      this.speed = 0;
    }
  }
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
        this.waypoints.push(new tr.app.pnp.waypoint(wp));
      }
    }

    if (this.waypoints.length > 0) {
      this.currentWaypoint = 0;
    }
  }

  this.getCurrentWaypoint = function() {
    return this.waypoints[this.currentWaypoint];
  }

  this.insertWaypoint = function() {
    var wp = [];
    console.log(this);
    for (var i = 0; i < this.waypoints.length; i++) {
      wp.push(this.waypoints[i]);
      if (i == this.currentWaypoint) {
        var config = Object.assign({}, this.waypoints[i].config);
        config.positions = Object.assign([], config.positions);
        wp.push(new tr.app.pnp.waypoint(config));
      }
    }

    if (wp.length == 0) {
      wp.push(new tr.app.pnp.waypoint());
      this.currentWaypoint = 0;
    } else {
      this.currentWaypoint += 1;
    }

    this.waypoints = wp;
  }

  this.removeWaypoint = function() {
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
  }

  this.incrementWaypoint = function(i) {
    this.currentWaypoint += i;
    if (this.currentWaypoint < 0) {
      this.currentWaypoint = 0;
    } else if (this.currentWaypoint >= this.waypoints.length) {
      this.currentWaypoint = this.waypoints.length - 1;
    }
  }

  this.setup();
}
