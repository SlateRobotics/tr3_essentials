tr.gui.chain = function(_p5) {
  this.p5 = _p5;
  if (!_p5) this.p5 = window;

  this.eef = {
    x: 0,
    y: 0,
    z: 0
  }
  this.chain = [];

  this.draw = function() {
    this.p5.push();
    this.p5.scale(0.2);

    this.p5.noStroke();
    this.p5.directionalLight(255, 255, 255, 500, -1250, -1250);
    this.p5.ambientLight(150);
    this.p5.ambientMaterial(40);

    this.eef.x = 0;
    this.eef.y = 0;
    this.eef.z = 0;

    if (this.chain.length == 7) {
      this.p5.push();
      this.p5.normalMaterial();
      this.p5.fill('green');
      this.p5.sphere(50);
      this.p5.pop();
    }

    for (var i = 0; i < this.chain.length; i++) {
      var link;
      var id;
      var meshId;
      var r_x;
      var r_y;
      var r_z;
      var x;
      var y;
      var z;
      var animate;

      if (this.chain[i][0]) {
        var chainLink = this.chain[i];

        link = chainLink[0];
        r_x = chainLink[1];
        r_y = chainLink[2];
        r_z = chainLink[3];
        x = chainLink[4];
        y = chainLink[5];
        z = chainLink[6];
        animate = chainLink[7];

      } else {
        link = this.chain[i].mesh;
        r_x = this.chain[i].rotate.x;
        r_y = this.chain[i].rotate.y;
        r_z = this.chain[i].rotate.z;
        x = this.chain[i].translate.x;
        y = this.chain[i].translate.y;
        z = this.chain[i].translate.z;
        this.chain[i].p5 = this.p5;
        animate = this.chain[i].animate.bind(this.chain[i]);
      }

      this.p5.translate(x, y, z);
      this.p5.rotateX(r_x);
      this.p5.rotateY(r_y);
      this.p5.rotateZ(r_z);

      if (animate) {
        var a = animate();
      }

      var r = {
        x: r_x,
        y: r_y,
        z: r_z
      };
      var v = {
        x: x,
        y: y,
        z: z
      };

      if (a) {
        r.x += a.x;
        r.y += a.y;
        r.z += a.z;
      }

      this.addEef(r, v);

      if (!link) continue;

      var mo = this.chain[i].link.meshOffset;
      if (mo) {
        this.p5.translate(mo.x, mo.y, mo.z);
        this.p5.model(link);
        this.p5.translate(-mo.x, -mo.y, -mo.z);
      } else {
        this.p5.model(link);
      }
    }

    this.p5.pop();
  }

  this.addEef = function(r, v) {
    //convert degrees to radians
    r.x *= Math.PI / 180.0;
    r.y *= Math.PI / 180.0;
    r.z *= Math.PI / 180.0;

    v = this.rotateX(v, r.x);
    //v = this.rotateY(v, r.y);
    //v = this.rotateZ(v, r.z);

    this.eef.x += v.x;
    this.eef.y += v.y;
    this.eef.z += v.z;
  }

  this.rotateX = function(v, x) {
    var v2 = Object.assign({}, v);
    v2.y = v.y * Math.cos(x) - v.z * Math.sin(x);
    v2.z = v.y * Math.sin(x) + v.z * Math.cos(x);
    return v2;
  }

  this.rotateY = function(v, y) {
    var v2 = Object.assign({}, v);
    v2.x = v.x * Math.cos(y) + v.z * Math.sin(y);
    v2.z = -v.x * Math.sin(y) + v.z * Math.cos(y);
    return v2;
  }

  this.rotateZ = function(v, z) {
    var v2 = Object.assign({}, v);
    v2.x = v.x * Math.cos(z) - v.y * Math.sin(z);
    v2.y = v.x * Math.sin(z) + v.y * Math.cos(z);
    return v2;
  }
}
