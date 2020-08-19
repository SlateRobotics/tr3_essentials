tr.gui.minimap = {
  defaults: function() {
    this.border = false;
    this.padding = 5;
    this.scale = 10;

    this.buttonRadius = 36;
    this.btnCancel = {};

    this.goal = '';
  },

  setup: function() {
    this.onClick = function() {
      this.componentConfig.handleClick_Goal.bind(this)();
      this.componentConfig.handleClick_Button(this.btnCancel, function() {
        var l = tr.data.nav.status.status_list;
        for (var i = 0; i < l.length; i++) {
          if (l[i].status == 1) {
            tr.data.socket.emit('/move_base/cancel', l[i].goal_id);
          }
        }
      }.bind(this)).bind(this)();
      this.componentConfig.handleClick_Button(this.btnZoomIn, function() {
        this.scale += 1;
        if (this.scale >= 20) this.scale = 20;
      }.bind(this)).bind(this)();
      this.componentConfig.handleClick_Button(this.btnZoomOut, function() {
        this.scale -= 1;
        if (this.scale <= 1) this.scale = 1;
      }.bind(this)).bind(this)();
    }
  },

  draw: function() {
    this.btnCancel = {
      x: 17,
      y: this.size.h - 25,
      r: 18
    };
    this.btnZoomIn = {
      x: this.size.w - 27,
      y: this.size.h - 65,
      r: 18
    };
    this.btnZoomOut = {
      x: this.size.w - 27,
      y: this.size.h - 25,
      r: 18
    }

    this.absolutePosition = this.getAbsolutePosition();
    this.componentConfig.drawBackground.bind(this)();
    this.componentConfig.drawLidar.bind(this)();
    //this.componentConfig.drawDepth.bind(this)();
    this.componentConfig.drawMap.bind(this)();
    this.componentConfig.drawGoal.bind(this)();
    this.componentConfig.drawButtons.bind(this)();
  },

  handleClick_Button: function(btn, callback) {
    return function() {
      var p = {
        x: this.absolutePosition.x + btn.x,
        y: this.absolutePosition.y + btn.y
      }

      var d = {
        x: (mouseX - p.x),
        y: (mouseY - p.y)
      }

      var dist = sqrt((d.x * d.x) + (d.y * d.y));

      if (dist < btn.r - 1) {
        callback();
      }
    }
  },

  handleClick_Goal: function() {
    var p = {
      x: this.absolutePosition.x + this.center.x,
      y: this.absolutePosition.y + this.center.y
    }

    var d = {
      x: (mouseX - p.x) / this.scale,
      y: -(mouseY - p.y) / this.scale
    }

    var _x = d.x * this.scale;
    var _y = d.y * this.scale;
    var dist = sqrt((_x * _x) + (_y * _y));
    if (dist < this.radius - 1) {
      if (!tr.data.odom) return;
      var a = tr.data.odom.orientation.z - 1.5708;
      var x = d.x * cos(a) - d.y * sin(a);
      var y = d.x * sin(a) + d.y * cos(a);

      this.goal = {
        position: {
          x: x + tr.data.odom.position.x,
          y: y + tr.data.odom.position.y,
          z: 0
        },
        orientation: {
          x: 0,
          y: 0,
          z: 0,
          w: 1
        }
      };

      tr.data.socket.emit('/move_base_simple/goal', {
        header: {
          seq: 0,
          stamp: {
            secs: 0,
            nsecs: 0
          },
          frame_id: 'map',
        },
        pose: this.goal,
      });
    }
  },

  drawGoal: function() {
    if (!this.goal || tr.data.nav.complete) return;
    if (!tr.data.odom) return;

    var p = tr.data.odom.position;

    translate(this.center.x, this.center.y);

    var d = this.goal.position;

    var a = -tr.data.odom.orientation.z + 1.5708;
    var x = (d.x - p.x) * cos(a) - (d.y - p.y) * sin(a);
    var y = (d.x - p.x) * sin(a) + (d.y - p.y) * cos(a);

    x *= this.scale;
    y *= this.scale;

    var dist = sqrt((x * x) + (y * y));
    if (dist < this.radius - 1) {
      stroke("red");
      fill("red");
      strokeWeight(2);
      line(x, -y, x, -y - 25);
      triangle(x, -y - 25, x, -y - 15, x + 10, -y - 20);
    }

    translate(-this.center.x, -this.center.y);
  },

  drawButton: function(btn, backColor, txt) {
    fill(backColor);
    stroke("rgb(50, 50, 50)");
    circle(btn.x, btn.y, btn.r * 2);

    stroke("white");
    fill("white");
    text(" " + txt, btn.x - btn.r, btn.y - btn.r, btn.r * 2, btn.r * 2);
  },

  drawButtons: function() {
    textFont(tr.fonts.noto);
    textSize(22);
    textAlign(CENTER, TOP);

    this.componentConfig.drawButton(this.btnZoomOut, "rgb(95, 95, 95)", "-");
    this.componentConfig.drawButton(this.btnZoomIn, "rgb(95, 95, 95)", "+");
    this.componentConfig.drawButton(this.btnCancel, "red", "x");
  },

  drawLidar: function() {
    var l = tr.data.lidar;
    if (!l.ranges) return;

    stroke("red");
    fill("red");
    strokeWeight(0.3);

    for (var i = 0; i < l.ranges.length; i++) {
      var m = l.ranges[i];
      if (m) {
        var a = l.angle_min + i * l.angle_increment + 1.5708;
        var x = sin(a) * m * this.scale;
        var y = cos(a) * m * this.scale;

        var d = sqrt((x * x) + (y * y));
        if (d < this.radius - 1) {
          point(this.center.x + x, this.center.y + y);
        }
      }
    }
  },

  drawDepth: function() {
    if (!tr.data.depth) return;

    stroke("orange");
    fill("orange");
    strokeWeight(0.3);

    for (var i = 0; i < tr.data.depth.length; i += 3) {
      var d = {
        x: tr.data.depth[i],
        y: tr.data.depth[i + 1],
        z: tr.data.depth[i + 2]
      }
      if (d.z > 0) {
        var x = d.x * this.scale;
        var y = d.y * this.scale;
        var dist = sqrt((x * x) + (y * y));
        if (dist < this.radius - 1) {
          point(this.center.x + x, this.center.y - y);
        }
      }
    }
  },

  drawMap: function() {
    if (!tr.data.map) return;
    if (!tr.data.odom) return;

    var p = tr.data.odom.position;

    translate(this.center.x, this.center.y);
    rotateZ(tr.data.odom.orientation.z - 1.5708);

    stroke("white");
    fill("white");
    strokeWeight(0.3);

    for (var i = 0; i < tr.data.map.length; i += 2) {
      var d = {
        x: tr.data.map[i],
        y: tr.data.map[i + 1]
      }
      var x = (d.x - p.x) * this.scale;
      var y = (d.y - p.y) * this.scale;
      var dist = sqrt((x * x) + (y * y));
      if (dist < this.radius - 1) {
        point(x, -y);
      }
    }

    rotateZ(-tr.data.odom.orientation.z + 1.5708);
    translate(-this.center.x, -this.center.y);
  },

  drawBackground: function() {
    var x = this.pos.x + this.size.w / 2.0 - this.padding;
    var y = this.pos.y + this.size.h / 2.0 - this.padding;

    this.center = {
      x: x,
      y: y
    };

    var d = this.size.w - this.padding * 2.0 - this.margin * 2.0;
    if (this.size.h < this.size.w) {
      d = this.size.h - this.padding * 2.0 - this.margin * 2.0;
    }

    this.radius = d / 2.0;

    stroke("rgb(50, 50, 50)");
    fill("rgb(100, 100, 100)");
    circle(x, y, d);

    var w = Math.floor(.725 * this.scale);
    stroke("white");
    fill("white");
    rect(x - w / 2, y - w / 2, w, w);
  },
};
