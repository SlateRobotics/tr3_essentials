if (!tr) var tr = {};
if (!tr.gui) tr.gui = {};

tr.gui.header = function(config) {
  this.id = config.id || "header";
  this.visible = true;
  this.config = Object.assign({}, config);
  this.pos = {
      x: 0,
      y: 0
    };
  this.size = config.size || {
    w: 1,
    h: 50
  };
  this.config.size = Object.assign({}, this.size);
  this.text = config.text || "New App";
  this.padding = config.padding || 10;
  this.parent = config.parent;

  if (this.size.w <= 1) {
    this.size.w = this.size.w * this.parent.size.w;
  }

  this.closeButton = {
    pos: { x: 0, y: 0 },
    size: { w: 0, h: 0 }
  };

  this.stopButton = {
    pos: { x: 0, y: 0 },
    size: { w: 0, h: 0 }
  };

  this.computeSize = function () {
    if (this.config.size.w == "fill") {
      var p = this.parent.translateState;
      this.size.w = this.parent.size.w - p.x;
    } else if (this.config.size.w <= 1) {
      var parentWidth = (this.parent.size.w - this.parent.margin * 2.0 - this.parent.padding * 2.0);
      this.size.w = this.config.size.w * parentWidth;
    }

    if (this.config.size.h == "fill") {
      var p = this.parent.translateState;
      this.size.h = this.parent.size.h - p.y;
    } else if (this.config.size.h <= 1) {
      var parentHeight = (this.parent.size.h - this.parent.margin * 2.0 - this.parent.padding * 2.0);
      this.size.h = this.config.size.h * parentHeight;
    }
  }

  this.draw = function() {
    this.computeSize();

    noStroke();
    fill(255);
    textSize(28)
    textAlign(LEFT, CENTER);
    text(this.text, this.padding, this.size.h / 2.0 + 3);

    this.drawCloseButton();
    this.drawStopButton();
  }

  this.drawCloseButton = function () {
    this.closeButton.size.w = this.size.h;
    this.closeButton.size.h = this.closeButton.size.w;
    this.closeButton.pos.x = this.size.w - this.closeButton.size.w;
    this.closeButton.pos.y = 0;

    fill("red");
    stroke("black");
    rect(this.closeButton.pos.x, this.closeButton.pos.y, this.closeButton.size.w, this.closeButton.size.h);

    fill(255);
    stroke(255);
    textAlign(CENTER, CENTER);
    textSize(40);

    fill(255);
    stroke(255);
    text("X", this.closeButton.pos.x + this.closeButton.size.w / 2.0, this.closeButton.pos.y + this.closeButton.size.h / 2.0 - 2);
  }

  this.drawStopButton = function () {
    this.stopButton.size.w = this.size.h;
    this.stopButton.size.h = this.stopButton.size.w;
    this.stopButton.pos.x = this.closeButton.pos.x - this.stopButton.size.w - 5;
    this.stopButton.pos.y = 0;

    noFill();
    noStroke();

    rect(this.stopButton.pos.x, this.stopButton.pos.y, this.stopButton.size.w, this.stopButton.size.h);

    fill(255);
    stroke(255);
    textAlign(CENTER, CENTER);
    textSize(20);
    var t = [this.stopButton.pos.x + this.stopButton.size.w / 2.0, this.stopButton.pos.y + this.stopButton.size.h / 2.0];

    if (tr.data.stopped == false) {
      translate(t[0], t[1]);
      rotate(PI/8);
      fill("white");
      this.polygon(0, 0, 22, 8);
      fill("red");
      this.polygon(0, 0, 20, 8);
      rotate(-PI/8);
      translate(-t[0], -t[1]);
    } else {
      translate(t[0], t[1]);
      rotate(PI/8);
      fill("white");
      circle(0, 0, 44);
      fill("green");
      circle(0, 0, 40);
      rotate(-PI/8);
      translate(-t[0], -t[1]);
    }
  }

  this.polygon = function (x, y, radius, npoints) {
    let angle = TWO_PI / npoints;
    beginShape();
    for (let a = 0; a < TWO_PI; a += angle) {
      let sx = x + cos(a) * radius;
      let sy = y + sin(a) * radius;
      vertex(sx, sy);
    }
    endShape(CLOSE);
  }

  this.mouseClicked = function() {
    //this.mousePressed();
  }

  this.mousePressed = function() {
    var x1 = mouseX > this.closeButton.pos.x;
    var x2 = mouseX < this.closeButton.pos.x + this.closeButton.size.w;
    var y1 = mouseY > this.closeButton.pos.y;
    var y2 = mouseY < this.closeButton.pos.y + this.closeButton.size.h;

    if (x1 && x2 && y1 && y2) {
      this.parent.app.close();
      this.parent.reset();
    }

    x1 = mouseX > this.stopButton.pos.x;
    x2 = mouseX < this.stopButton.pos.x + this.stopButton.size.w;
    y1 = mouseY > this.stopButton.pos.y;
    y2 = mouseY < this.stopButton.pos.y + this.stopButton.size.h;

    if (x1 && x2 && y1 && y2) {
      if (tr.data.stopped == false) {
        tr.data.stopped = true;
        tr.data.socket.emit("/tr3/stop", true);
      } else {
        tr.data.stopped = false;
        tr.data.socket.emit("/tr3/stop", false);
      }
    }
  }

  this.mouseReleased = function() {
    // to do
  }

  this.mouseDragged = function() {
    // to do
  }
}
