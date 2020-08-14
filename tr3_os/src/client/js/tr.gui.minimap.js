tr.gui.minimap = {
  defaults: function() {
    this.border = false;
    this.padding = 5;
    this.scale = 10;
  },

  setup: function () {
    this.onClick = function () {
      var r = tr.data.odom.position;

      var p = this.absolutePosition;
      p.x += this.center.x;
      p.y += this.center.y;

      var d = {
        x: (mouseX - p.x) / this.scale,
        y: -(mouseY - p.y) / this.scale
      }

      d.x += r.x;
      d.y += r.y;

      tr.data.socket.emit('/move_base_simple/goal', {
        header: {
          seq: 0,
          stamp: {
            secs: 0,
            nsecs: 0
          },
          frame_id: 'map',
        },
        pose: {
          position: {
            x: d.x,
            y: d.y,
            z: 0
          },
          orientation: {
            x: 0,
            y: 0,
            z: 0,
            w: 1
          }
        }
      });
    }
  },

  draw: function() {
    this.absolutePosition = this.getAbsolutePosition();
    this.componentConfig.drawBackground.bind(this)();
    this.componentConfig.drawLidar.bind(this)();
    this.componentConfig.drawDepth.bind(this)();
    this.componentConfig.drawMap.bind(this)();
  },

  drawLidar: function () {
    var l = tr.data.lidar;
    if (!l.ranges) return;

    stroke("red");
    fill("red");

    for (var i = 0; i < l.ranges.length; i++) {
      var m = l.ranges[i];
      if (m) {
        var a = l.angle_min + i * l.angle_increment + 1.5708;
        var x = sin(a) * m * this.scale;
        var y = cos(a) * m * this.scale;

        var d = sqrt((x * x) + (y * y));
        if (d < this.radius - 1) {
          circle(this.center.x + x, this.center.y + y, 1);
        }
      }
    }
  },

  drawDepth: function () {
    if (!tr.data.depth) return;

    stroke("orange");
    fill("orange");

    for (var i = 0; i < tr.data.depth.length; i++) {
      var d = tr.data.depth[i];
      if (d.z > 0) {
        var x = d.x * this.scale;
        var y = d.y * this.scale;
        var dist = sqrt((d.x * d.x) + (d.y * d.y));
        if (dist < this.radius - 1) {
          circle(this.center.x + x, this.center.y - y, 1);
        }
      }
    }
  },

  drawMap: function () {
    if (!tr.data.map) return;
    if (!tr.data.odom) return;

    var p = tr.data.odom.position;

    translate(this.center.x, this.center.y);
    rotateZ(tr.data.odom.orientation.z);

    stroke("white");
    fill("white");

    for (var i = 0; i < tr.data.map.length; i++) {
      var d = tr.data.map[i];
      var x = (d.x - p.x) * this.scale;
      var y = (d.y - p.y) * this.scale;
      var dist = sqrt((x * x) + (y * y));
      if (dist < this.radius - 1) {
        circle(x, -y, 1);
      }
    }

    rotateZ(-tr.data.odom.orientation.z);
    translate(-this.center.x, -this.center.y);
  },

  drawBackground: function () {
    var x = this.pos.x + this.size.w / 2.0 - this.padding;
    var y = this.pos.y + this.size.h / 2.0 - this.padding;

    this.center = { x: x, y: y };

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
