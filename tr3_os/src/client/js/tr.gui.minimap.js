tr.gui.minimap = {
  defaults: function() {
    this.border = false;
    this.padding = 5;
    this.scale = 10;
  },

  draw: function() {
    this.componentConfig.drawBackground.bind(this)();
    this.componentConfig.drawLidar.bind(this)();
    this.componentConfig.drawDepth.bind(this)();
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

    stroke("white");
    fill("white");

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
