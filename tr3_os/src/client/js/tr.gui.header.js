if (!tr) var tr = {};
if (!tr.gui) tr.gui = {};

tr.gui.header = function(config) {
  this.id = config.id || "header";
  this.visible = true;
  this.pos = {
      x: 0,
      y: 0
    },
    this.size = config.size || {
      w: 1,
      h: 50
    };
  this.text = config.text || "New App";
  this.padding = config.padding || 10;
  this.parent = config.parent;

  if (this.size.w <= 1) {
    this.size.w = this.size.w * this.parent.size.w;
  }

  this.closeButton = {
    pos: {
      x: 0,
      y: 0
    },
    size: {
      w: 0,
      h: 0
    }
  };

  this.draw = function() {
    noStroke();
    fill(255);
    textSize(28)
    textAlign(LEFT, CENTER);
    text(this.text, this.padding, this.size.h / 2.0 + 3);

    this.closeButton.size.w = 30;
    this.closeButton.size.h = this.closeButton.size.w;
    this.closeButton.pos.x = this.size.w - this.padding - this.closeButton.size.w;
    this.closeButton.pos.y = this.padding;

    fill("red");
    stroke("red");
    rect(this.closeButton.pos.x, this.closeButton.pos.y, this.closeButton.size.w, this.closeButton.size.h);

    fill(255);
    stroke(255);
    textAlign(CENTER, CENTER);
    textSize(20);
    text("X", this.size.w - this.padding - this.closeButton.size.w / 2.0, this.padding + this.closeButton.size.w / 2.0);
  }

  this.mouseClicked = function() {
    this.mousePressed();
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
  }

  this.mouseReleased = function() {
    // to do
  }

  this.mouseDragged = function() {
    // to do
  }
}
