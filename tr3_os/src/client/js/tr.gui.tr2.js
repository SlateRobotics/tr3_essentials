var linkNames = ["b0", "b1", "b2", "b3", "a0", "a1", "a2", "a3", "g0", "g1", "h0", "h1"];
var models = {};

tr.gui.tr2 = {
  defaults: function() {
    this.softAlign = true;
    this.align = {
      v: "CENTER",
      h: "CENTER"
    };
    this.border = false;
    this.useLiveState = true;

    this.displayMap = false;
    this.displayLidar = false;
    this.displayPointCloud = false;

    this.links = {};

    this.tr2 = new tr.lib.tr2();

    this.state = {};
    this.state.g0 = 0;
    this.state.h0 = 0;
    this.state.h1 = 0;

    this.allowDrag = false;
    this.p5 = '';
    this.container = '';

    this.cameraPosLast = {
      x: '',
      y: '',
      z: ''
    };
    this.cameraRadius = 300;
    this.cameraPosDefault = {
      x: this.cameraRadius,
      y: 0,
      z: 200
    };
    this.cameraDif = {
      x: 74.5,
      y: 0,
      z: 0.50
    };
    this.cameraPos = {
      x: this.cameraPosDefault.x,
      y: this.cameraPosDefault.y,
      z: this.cameraPosDefault.z
    };
    if (this.config.cameraPos) {
      this.cameraPos = this.config.cameraPos
    }
  },

  setup: function() {
    this.container = document.createElement('div');
    this.container.id = "tr2-render-" + Math.floor(Math.random() * 1000000);
    this.container.style.position = "absolute";
    this.container.style.left = this.pos.x;
    this.container.style.top = this.pos.y;
    this.container.style.display = "none";
    document.body.appendChild(this.container);

    if (this.config.useLiveState != undefined) {
      this.useLiveState = this.config.useLiveState;
    }

    if (this.config.displayLidar != undefined) {
      this.displayLidar = this.config.displayLidar;
    }

    if (this.config.displayMap != undefined) {
      this.displayMap = this.config.displayMap;
    }

    if (this.config.displayPointCloud != undefined) {
      this.displayPointCloud = this.config.displayPointCloud;
    }

    this.zoomOut = function () {
      this.cameraRadius += 25;
    }

    this.zoomIn = function () {
      this.cameraRadius -= 25;
    }

    this.children.push(new tr.gui.component().setup({
      id: "btn-zoom-in",
      type: "container",
      parent: this,
      background: "rgb(150,150,150)",
      size: { w: 40, h: 40 },
      pos: { x: 10, y: -95 },
      posType: "fixed",
      radius: 20,
      onClick: function () {
        this.parent.zoomIn();
      },
      children: [{
        type: "text",
        text: "+",
        align: { v: "CENTER", h: "CENTER" },
      }],
    }));

    this.children.push(new tr.gui.component().setup({
      id: "btn-zoom-out",
      type: "container",
      parent: this,
      background: "rgb(150,150,150)",
      size: { w: 40, h: 40 },
      pos: { x: 10, y: -50 },
      posType: "fixed",
      radius: 20,
      onClick: function () {
        this.parent.zoomOut();
      },
      children: [{
        type: "text",
        text: "-",
        align: { v: "CENTER", h: "CENTER" },
      }],
    }));

    this.p5 = new p5(function(p) {}, this.container.id);
    this.p5.createCanvas(this.size.w, this.size.h, WEBGL);

    this.p5.angleMode(RADIANS);
    this.p5.perspective();

    this.links.b0 = tr.links.b0;
    this.links.w0 = tr.links.w0;
    this.links.a0 = tr.links.a0;
    this.links.a1 = tr.links.a1;
    this.links.a2 = tr.links.a2;
    this.links.a3 = tr.links.a3;
    this.links.a4 = tr.links.a4;
    this.links.g0 = tr.links.g0;
    this.links.g1 = tr.links.g1;
    this.links.h0 = tr.links.h0;
    this.links.h1 = tr.links.h1;
  },

  draw: function() {
    if (this.p5.height != this.parent.size.h) {
      this.pos = this.parent.pos;
      this.size = this.parent.size;

      this.p5.remove();
      this.container.remove();

      this.container = document.createElement('div');
      this.container.id = "tr2-render-" + Math.floor(Math.random() * 1000000);
      this.container.style.position = "absolute";
      this.container.style.left = this.pos.x;
      this.container.style.top = this.pos.y;
      this.container.style.display = "none";
      document.body.appendChild(this.container);

      this.p5 = new p5(function(p) {}, this.container.id);
      this.p5.createCanvas(this.size.w, this.size.h, WEBGL);

      this.p5.angleMode(RADIANS);
      this.p5.perspective();

      this.links.b0 = tr.links.b0;
      this.links.w0 = tr.links.w0;
      this.links.a0 = tr.links.a0;
      this.links.a1 = tr.links.a1;
      this.links.a2 = tr.links.a2;
      this.links.a3 = tr.links.a3;
      this.links.a4 = tr.links.a4;
      this.links.g0 = tr.links.g0;
      this.links.g1 = tr.links.g1;
      this.links.h0 = tr.links.h0;
      this.links.h1 = tr.links.h1;
    }

    if (this.useLiveState) {
      var s = tr.data.robotState;
      for (var i = 0; i < s.name.length; i++) {
        this.state[s.name[i]] = s.position[i];
      }
    }

    var absPos = this.getAbsolutePosition();

    this.container.style.display = "block";
    this.container.style.left = absPos.x;
    this.container.style.top = absPos.y;

    this.p5.clear();

    var mapRadius = 100;
    var lon = this.cameraDif.x * 100.0 / mapRadius;
    var lat = 2 * Math.atan(Math.exp(this.cameraDif.z * 100.0 / mapRadius)) - Math.PI / 2;

    this.cameraPos.x = this.cameraRadius * Math.cos(lat) * Math.cos(lon);
    this.cameraPos.y = this.cameraRadius * Math.cos(lat) * Math.sin(lon);
    this.cameraPos.z = this.cameraRadius * Math.sin(lat) + 100;

    this.p5.camera(this.cameraPos.x, this.cameraPos.y, this.cameraPos.z, 0, 0, 100, 0, 0, -1);

    tr.gui.drawLink(this.links["b0"], 1.5708, 3.1415, 0, 0, 0, 0, null, this.p5);
    tr.gui.drawLink(this.links["w0"], 0, 1.5708, 0, 328.1, 0, 0, null, this.p5);
    tr.gui.drawLink(this.links["w0"], 0, -1.5708, 0, -328.1, 0, 0, null, this.p5);

    var arm = new tr.gui.chain(this.p5);
    for (var i = 0; i < this.tr2.arm.length; i++) {
      var link = this.tr2.arm[i];
      arm.chain.push({
        id: link.id,
        axis: link.axis,
        mesh: this.links[link.meshId],
        link: link,
        state: this.state,
        rotate: link.rotate,
        translate: link.translate,
        animate: function() {
          if (this.link.fixed) return;
          var f = 1.0;
          if (this.link.flip) f = -1.0;
          this.p5["rotate" + this.axis](this.link.offset * f);
          this.p5["rotate" + this.axis](this.state[this.link.id] * f);
        }
      });
    }

    /*arm.chain.push([this.links["g1"], -90, 0, 90, 0, -7.5, 135, function() {
      this.p5.translate(0, -5);
      this.p5.translate(0, this.state.g0 / 100.0 * -40);
    }.bind(this)]);
    arm.chain.push([this.links["g1"], 0, 0, 0, 0, 7.5, 0, function() {
      this.p5.translate(0, 10);
      this.p5.translate(0, this.state.g0 / 100.0 * 80);
    }.bind(this)]);*/
    arm.draw();

    var head = new tr.gui.chain(this.p5);
    for (var i = 0; i < this.tr2.head.length; i++) {
      var link = this.tr2.head[i];
      head.chain.push({
        id: link.id,
        axis: link.axis,
        mesh: this.links[link.meshId],
        link: link,
        state: this.state,
        rotate: link.rotate,
        translate: link.translate,
        animate: function() {
          var f = 1.0;
          if (this.link.flip) f = -1.0;
          this.p5["rotate" + this.axis](this.link.offset * f);
          this.p5["rotate" + this.axis](this.state[this.link.id] * f);
        }
      });
    }
    head.draw();

    this.p5.rotateZ(3.1415);

    /*this.p5.strokeWeight(2);
    this.p5.stroke('red');
    this.p5.line(0, 0, 0, -1000, 0, 0);

    this.p5.stroke('green');
    this.p5.line(0, 0, 0, 0, 1000, 0);

    this.p5.stroke('blue');
    this.p5.line(0, 0, 0, 0, 0, 1000);*/

    if (this.displayLidar) {
      this.componentConfig.drawLidar.bind(this)();
    }

    if (this.displayMap) {
      this.componentConfig.drawMap.bind(this)();
    }

    if (this.displayPointCloud) {
      this.componentConfig.drawPointCloud.bind(this)();
    }

    //this.buttonZoomOut.position = { x: 25, y: this.size.h - 25 };
    //this.buttonZoomOut.draw();
  },

  drawMap: function () {
    if (!tr.data.map) return;
    if (!tr.data.odom) return;

    var p = tr.data.odom.position;

    this.p5.rotateZ(tr.data.odom.orientation.z - 1.5708);

    this.p5.stroke("white");

    for (var i = 0; i < tr.data.map.length; i += 2) {
      var d = {
        x: tr.data.map[i],
        y: tr.data.map[i + 1]
      }
      var x = (d.x - p.x) * 200.0;
      var y = (d.y - p.y) * 200.0;
      this.p5.translate(-x, y, 0);
      this.p5.sphere(4);
      this.p5.translate(x, -y, 0);
    }

    this.p5.rotateZ(-tr.data.odom.orientation.z + 1.5708);
  },

  drawLidar: function () {
    var l = tr.data.lidar;
    if (!l.ranges) return;

    this.p5.stroke("red");

    for (var i = 0; i < l.ranges.length; i++) {
      var m = l.ranges[i];
      if (m) {
        var a = l.angle_min + i * l.angle_increment + 1.5708;
        var x = sin(a) * m * 200;
        var y = cos(a) * m * 200;
        this.p5.translate(-x, -y, 0);
        this.p5.sphere(2);
        this.p5.translate(x, y, 0);
      }
    }
  },

  drawPointCloud: function () {
    this.p5.stroke("white");
    for (var i = 0; i < tr.data.depth.length; i += 3) {
      var d = {
        x: tr.data.depth[i],
        y: tr.data.depth[i + 1],
        z: tr.data.depth[i + 2]
      }
      this.p5.translate(-d.x * 200, d.y * 200, d.z * 200);
      this.p5.sphere(4);
      this.p5.translate(d.x * 200, -d.y * 200, -d.z * 200);
    }
  },

  clear: function() {
    this.p5.clear();
    this.p5.remove();
  },

  mousePressed: function() {
    this.cameraPosLast.x = '';
    this.cameraPosLast.y = '';
    this.allowDrag = true;
  },

  mouseReleased: function() {
    this.allowDrag = false;
  },

  mouseDragged: function() {
    if (!this.allowDrag) {
      return;
    }

    if (!this.cameraPosLast.x) {
      this.cameraPosLast.x = mouseX;
    }

    if (!this.cameraPosLast.y) {
      this.cameraPosLast.y = mouseY;
    }

    this.cameraDif.x -= (this.cameraPosLast.x - mouseX) / 100;
    this.cameraDif.z -= (this.cameraPosLast.y - mouseY) / 100;

    this.cameraPosLast.x = mouseX;
    this.cameraPosLast.y = mouseY;
  },
}
