if (!tr) var tr = {};
if (!tr.gui) tr.gui = {};

tr.gui.input = function(config) {
  this.id = config.id;
  this.parent = config.parent;
  this.config = config;
  this.type = "input";

  this.element = '';
  this.pos = config.pos || {
    x: 0,
    y: 0
  };
  this.size = config.size || {
    w: 1,
    h: 0
  };
  this.align = config.align || {
    v: "TOP",
    h: "LEFT"
  };

  this.textSize = config.textSize || 16;
  this.margin = config.margin || 0;

  this.translateState = {
    x: 0,
    y: 0
  };

  this.onKeyPress = config.onKeyPress;
  this.onClick = config.onClick;

  this.setup = function() {
    if (this.size.w <= 1) {
      this.size.w = this.size.w * (this.parent.size.w - this.parent.margin * 2.0 - this.parent.padding * 2.0);
    }

    if (this.size.h == 0) {
      this.size.h = this.textSize + this.margin * 2;
    } else if (this.size.h <= 1) {
      this.size.h = this.size.h * (this.parent.size.h - this.parent.margin * 2.0 - this.parent.padding * 2.0);
    }

    this.element = createInput('');
    this.element.hide();
    this.element.input(this.keyPressed);
  }

  this.draw = function() {
    this.translate(this.pos.x, this.pos.y);

    noStroke();
    fill(255);
    textSize(this.textSize);
    textAlign(window[this.align.h], window[this.align.v]);

    this.translate(this.padding, this.padding);

    var pos = this.getAbsolutePosition();

    this.element.position(pos.x, pos.y);
    this.element.size(this.size.w - this.padding * 2, this.size.h - this.padding * 2);
    this.element.show();

    this.translate(-this.padding, -this.padding);
    this.translate(-this.pos.x, -this.pos.y);
  }

  this.getAbsolutePosition = function() {
    var x = this.translateState.x;
    var y = this.translateState.y;

    var component = this;
    while (component.parent) {
      x += component.parent.translateState.x;
      y += component.parent.translateState.y;
      component = component.parent;
    }

    return {
      x: x,
      y: y
    };
  }

  this.translate = function(x, y) {
    this.translateState.x += x;
    this.translateState.y += y;
    translate(x, y);
  }

  this.keyPressed = function() {
    var val = this.value();

    if (this.onKeyPress) {
      this.onKeyPress(val);
    }

    console.log(val);
  }

  this.mousePressed = function() {
    if (this.onClick) {
      console.log(mouseX, mouseY);
    }
  }

  this.setup();
}
