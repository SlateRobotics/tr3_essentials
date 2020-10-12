if (!tr) var tr = {};
if (!tr.gui) tr.gui = {};

tr.gui.page = function(config) {
  this.setup = function(config) {
    this.id = config.id;
    this.config = Object.assign({}, config);
    this.pos = config.pos || {
      x: 0,
      y: 0
    };
    this.size = config.size || {
      w: 1.0,
      h: 1.0
    };
    this.config.size = Object.assign({}, this.size);

    this.margin = 0;
    this.padding = 0;
    this.header = {};
    this.children = [];
    this.app = config.app;
    this.background = config.background || "rgb(55, 55, 55)";
    this.translateState = {
      x: 0,
      y: 0
    };
    this.onDraw = config.onDraw;

    if (this.size.w <= 1) {
      this.size.w = this.size.w * canvasWidth;
    }

    if (this.size.h <= 1) {
      this.size.h = this.size.h * canvasHeight;
    }

    if (this.config.header) {
      this.config.header.id = "header";
      this.config.header.parent = this;
      this.children.push(new tr.gui.header(this.config.header));
    }

    var container = new tr.gui.component(tr.gui.container);
    container.setup({
      id: "body",
      parent: this,
      size: {
        w: 1.0,
        h: "fill"
      },
    });
    this.children.push(container);

    var bodyIdx = this.children.length - 1;
    if (!this.config.children) {
      this.config.children = [];
    }
    for (var i = 0; i < this.config.children.length; i++) {
      this.config.children[i].parent = this.children[bodyIdx];
      var child = tr.gui[this.config.children[i].type];
      if (typeof child == "function") {
        child = new child(this.config.children[i]);
      } else {
        child = new tr.gui.component(tr.gui[this.config.children[i].type]);
        child.setup(this.config.children[i]);
      }
      this.children[bodyIdx].children.push(child);
    }
  }

  this.reset = function() {
    this.setup(this.config);
  }

  this.computeSize = function () {
    if (this.config.size.w <= 1) {
      this.size.w = this.config.size.w * canvasWidth;
    }

    if (this.config.size.h <= 1) {
      this.size.h = this.config.size.h * canvasHeight;
    }
  },

  this.draw = function() {
    background(this.background);

    this.computeSize();

    this.drawChildren();

    if (this.onDraw) {
      this.onDraw();
    }
  }

  this.getApp = function() {
    return this.app
  }

  this.hideElements = function(p) {
    if (!p) p = this;
    if (!p.children) p.children = [];
    for (var i = 0; i < p.children.length; i++) {
      if (p.children[i].p5 && p.children[i].container) {
        p.children[i].container.style.display = "none";
      } else if (p.children[i].element) {
        p.children[i].element.hide();
      } else {
        this.hideElements(p.children[i]);
      }
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
      } else if (i > 0) {
        this.translate(-shift.x, this.children[i - 1].size.h);
        shift.x = 0;
        shift.y += this.children[i - 1].size.h;
        this.children[i].draw();
        this.translate(0, this.children[i].size.h);
        shift.y += this.children[i].size.h;
      } else {
        this.children[i].draw();
      }
    }
    this.translate(-shift.x, -shift.y);
  }

  this.translate = function(x, y) {
    this.translateState.x += x;
    this.translateState.y += y;
    translate(x, y);
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

  this.mouseClicked = function() {
    for (var i = 0; i < this.children.length; i++) {
      this.children[i].mouseClicked();
    }
  }

  this.mousePressed = function() {
    for (var i = 0; i < this.children.length; i++) {
      this.children[i].mousePressed();
    }
  }

  this.mouseReleased = function() {
    for (var i = 0; i < this.children.length; i++) {
      this.children[i].mouseReleased();
    }
  }

  this.mouseDragged = function() {
    for (var i = 0; i < this.children.length; i++) {
      this.children[i].mouseDragged();
    }
  }

  this.setup(config);
}
