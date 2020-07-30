if (!tr) tr = {};
if (!tr.gui) tr.gui = {};

tr.gui.component = function(componentConfig) {
  this.componentConfig = componentConfig;

  this.setup = function(config) {
    this.initialized = false;
    this.config = Object.assign({}, config);

    this.index = config.index || 0;
    this.id = this.config.id;
    this.parent = config.parent;
    this.pos = config.pos || {
      x: 0,
      y: 0
    };
    this.size = config.size || {
      w: 1,
      h: 1
    };
    this.margin = config.margin || 0;
    this.padding = config.padding || 0;
    this.type = "container";
    this.softAlign = true;
    this.align = config.align || {
      v: "TOP",
      h: "CENTER"
    };
    this.border = config.border;
    this.background = config.background || "rgba(0,0,0,0)";
    this.text = config.text || "";
    this.textSize = config.textSize || 22;
    this.textColor = config.textColor || 255;
    this.radius = config.radius || 0;
    this.children = [];
    this.translateState = {
      x: 0,
      y: 0
    };
    this.translateParent = {
      x: 0,
      y: 0
    };

    this.onChange = config.onChange;
    this.onClick = config.onClick;
    this.onDraw = config.onDraw;

    if (this.componentConfig.defaults) {
      this.componentConfig.defaults.bind(this)();
    }

    var parentWidth = (this.parent.size.w - this.parent.margin * 2.0 - this.parent.padding * 2.0);
    var parentHeight = (this.parent.size.h - this.parent.margin * 2.0 - this.parent.padding * 2.0)

    if (this.size.w == "fill") {
      var offset = 0;
      var shift = {
        x: 0,
        y: 0
      };

      for (var i = 0; i < this.parent.children.length; i++) {
        if (shift.x + this.parent.children[i].size.w <= this.parent.size.w) {
          shift.x += this.parent.children[i].size.w;
          offset += shift.x;
        } else {
          shift.x = this.parent.children[i].size.w;
          shift.y += this.parent.children[i - 1].size.h;
          offset = shift.x;
        }
      }
      this.size.w = parentWidth - offset;
    } else if (this.size.w <= 1) {
      this.size.w = this.size.w * parentWidth;
    }

    if (this.size.h == "fill") {
      var offset = 0;
      var shift = {
        x: 0,
        y: 0
      }

      for (var i = 0; i < this.parent.children.length; i++) {
        if (shift.x + this.parent.children[i].size.w <= this.parent.size.w) {
          shift.x += this.parent.children[i].size.w;
        } else {
          shift.x = this.parent.children[i].size.w;
          shift.y += this.parent.children[i - 1].size.h;
          offset = shift.y;
        }
      }

      if (shift.x >= parentWidth) {
        offset = this.parent.children[this.parent.children.length - 1].size.h;
      }

      this.size.h = parentHeight - offset;
    } else if (this.size.h <= 1) {
      this.size.h = this.size.h * parentHeight;
    }

    if (!this.config.children) {
      this.config.children = [];
    }
    for (var i = 0; i < this.config.children.length; i++) {
      this.config.children[i].parent = this;
      this.config.children[i].index = i;

      var child = tr.gui[this.config.children[i].type];
      if (typeof child == "function") {
        child = new child(this.config.children[i]);
      } else {
        child = new tr.gui.component(tr.gui[this.config.children[i].type]);
        child.setup(this.config.children[i]);
      }

      this.children.push(child);
    }

    if (this.softAlign == true && this.align.v == "CENTER") {
      this.pos.y = (this.parent.size.h - this.parent.padding * 2.0 - this.parent.margin * 2.0 - this.size.h) / 2.0;
    }

    if (this.border == null) {
      this.border = true;
    }

    if (this.componentConfig.setup) {
      this.componentConfig.setup.bind(this)();
    }

    this.initialized = true;
  }

  this.reset = function() {
    this.setup(this.config);
  }

  this.draw = function() {
    stroke(0);
    fill(this.background);

    if (this.size.w == "fill") {
      var parentWidth = (this.parent.size.w - this.parent.margin * 2.0 - this.parent.padding * 2.0);
      this.size.w = parentWidth - this.parent.translateState.x;
    }

    if (this.size.h == "fill") {
      var parentHeight = (this.parent.size.h - this.parent.margin * 2.0 - this.parent.padding * 2.0)
      this.size.h = parentHeight - this.parent.translateState.y;
    }

    var s = {};
    s.w = this.size.w - this.margin * 2.0;
    s.h = this.size.h - this.margin * 2.0;

    this.translateParent.x = this.parent.translateState.x;
    this.translateParent.y = this.parent.translateState.y;

    this.translate(this.pos.x, this.pos.y);
    this.translate(this.margin, this.margin);

    if (this.border == true) {
      tr.gui.rect(0, 0, s.w, s.h, this.radius);
    }

    this.translate(this.padding, this.padding);

    if (this.componentConfig.draw) {
      push();
      this.componentConfig.draw.bind(this)();
      pop();
    }

    this.drawChildren();

    this.translate(-this.padding, -this.padding);
    this.translate(-this.margin, -this.margin);
    this.translate(-this.pos.x, -this.pos.y);

    if (this.onDraw) {
      this.onDraw();
    }
  }

  this.drawChildren = function() {
    var shift = {
      x: 0,
      y: 0
    }
    for (var i = 0; i < this.children.length; i++) {
      if (shift.x + this.children[i].size.w <= this.size.w) {
        this.children[i].draw();
        this.translate(this.children[i].size.w, 0);
        shift.x += this.children[i].size.w;
      } else {
        this.translate(-shift.x, this.children[i - 1].size.h);
        shift.x = 0;
        shift.y += this.children[i - 1].size.h;
        this.children[i].draw();
        this.translate(this.children[i].size.w, 0);
        shift.x += this.children[i].size.w;
      }
    }
    this.translate(-shift.x, -shift.y);
  }

  this.translate = function(x, y, z) {
    this.translateState.x += x;
    this.translateState.y += y;
    this.translateState.z += z;
    translate(x, y, z);
  }

  this.getChild = function(id) {
    for (var i = 0; i < this.children.length; i++) {
      if (this.children[i].id == id) {
        return this.children[i];
      }

      if (this.children[i].children) {
        var child = this.children[i].getChild(id);
        if (child) {
          return child;
        }
      }
    }
  }

  this.getApp = function() {
    var tp = this.parent;
    while (tp.parent) {
      tp = tp.parent;
    }
    return tp.app;
  }

  this.getPage = function() {
    var tp = this.parent;
    while (tp.parent) {
      tp = tp.parent;
    }
    return tp;
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

  this.mouseClicked = function() {
    //this.mousePressed();
  }

  this.mousePressed = function() {
    for (var i = 0; i < this.children.length; i++) {
      if (this.children[i].mousePressed()) {
        return;
      }
    }

    if (!this.childClicked && (this.onClick || this.componentConfig.mousePressed)) {
      var t = {
        x: this.pos.x,
        y: this.pos.y
      };
      var selected = this;
      while (selected.parent) {
        t.x += selected.translateParent.x;
        t.y += selected.translateParent.y;
        selected = selected.parent;
      }

      var mX = mouseX - t.x;
      var mY = mouseY - t.y;

      var x1 = mX > 0;
      var x2 = mX < this.size.w;
      var y1 = mY > 0;
      var y2 = mY < this.size.h;

      if (x1 && x2 && y1 && y2) {
        if (this.onClick) {
          this.onClick();
        }
        if (this.componentConfig.mousePressed) {
          this.componentConfig.mousePressed.bind(this)();
        }
        return true;
      }
    }

    return false;
  }

  this.mouseReleased = function() {
    for (var i = 0; i < this.children.length; i++) {
      if (this.children[i].mouseReleased()) {
        return;
      }
    }

    if (this.componentConfig.mouseReleased) {
      this.componentConfig.mouseReleased.bind(this)();
    }
  }

  this.mouseDragged = function() {
    for (var i = 0; i < this.children.length; i++) {
      if (this.children[i].mouseDragged()) {
        return;
      }
    }

    if (this.componentConfig.mouseDragged) {
      this.componentConfig.mouseDragged.bind(this)();
    }
  }
}
