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

    this.links = {};
    this.links.b0 = '';
    this.links.b1 = '';
    this.links.b2 = '';
    this.links.b3 = '';
    this.links.a0 = '';
    this.links.a1 = '';
    this.links.a2 = '';
    this.links.a3 = '';
    this.links.g0 = '';
    this.links.g1 = '';
    this.links.h0 = '';
    this.links.h1 = '';

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

    this.p5 = new p5(function(p) {}, this.container.id);
    this.p5.createCanvas(this.size.w, this.size.h, WEBGL);

    this.p5.angleMode(DEGREES);
    this.p5.perspective();
    for (l in this.links) {
      this.links[l] = this.p5.loadModel("/stl/link_" + l + ".stl");
    }
  },

  draw: function() {
    var s = tr.data.robotState;
    for (var i = 0; i < s.name.length; i++) {
      this.state[s.name[i]] = s.position[i];
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

    tr.gui.drawLink(this.links["b0"], 90, -90, 0, 0, 0, 0, null, this.p5);
    tr.gui.drawLink(this.links["b1"], 0, 0, 0, 301.9, -150.2, 50.8, function() {
      this.p5.rotateX(this.p5.frameCount);
    }.bind(this), this.p5);
    tr.gui.drawLink(this.links["b1"], 0, 0, 0, -301.9, -150.2, 50.8, function() {
      this.p5.rotateX(this.p5.frameCount);
    }.bind(this), this.p5);

    var arm = new tr.gui.chain(this.p5);
    for (var i = 0; i < this.tr2.arm.length; i++) {
      var link = this.tr2.arm[i];

      arm.chain.push({
        id: link.id,
        mesh: this.links[link.meshId],
        link: link,
        state: this.state,
        rotate: link.rotate,
        translate: link.translate,
        animate: function() {
          this.p5.rotateY(this.link.offset);
          this.p5.rotateY(this.state[this.link.id] * 57.2958);
        }
      });
    }

    arm.chain.push([this.links["g1"], -90, 0, 90, 0, -7.5, 135, function() {
      this.p5.translate(0, -5);
      this.p5.translate(0, this.state.g0 / 100.0 * -40);
    }.bind(this)]);
    arm.chain.push([this.links["g1"], 0, 0, 0, 0, 7.5, 0, function() {
      this.p5.translate(0, 10);
      this.p5.translate(0, this.state.g0 / 100.0 * 80);
    }.bind(this)]);
    arm.draw();

    var head = new tr.gui.chain(this.p5);
    head.chain.push([this.links["h0"], -90, 0, 180, 0, 907.1, 122.7, function() {
      this.p5.rotateY(this.state.h0 * 57.2958);
    }.bind(this)]);
    head.chain.push([this.links["h1"], 180, 0, 0, 68.8, -161.4, 174.5, function() {
      this.p5.rotateX(-20);
      this.p5.rotateX(this.state.h1 * 57.2958);
    }.bind(this)]);
    head.draw();
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
