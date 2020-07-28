var app = {};

function App(config) {
  this.id = config.id;
  this.name = config.name;

  this.iconImg = ""; // p5.js img
  this.iconImgMask = ""; //p5.js mask

  this.config = config;
  this._configDraw = config.draw;
  this._configDrawIcon = config.drawIcon;
  this._configIconUrl = config.iconUrl;
  this._configMouseClicked = config.mouseClicked;
  this._configMousePressed = config.mousePressed;
  this._configMouseReleased = config.mouseReleased;
  this._configKeyPressed = config.keyPressed;
  this._configSetup = config.setup;
  config._app = this;

  this.pageCurrent = -1;
  this.pages = [];

  this.setup = function(id) {
    if (!this.id) {
      this.id = id;
    }

    if (this._configIconUrl) {
      this.iconImg = loadImage(this._configIconUrl);
    }

    this.pageCurrent = 0;
    if (this.config.pages) {
      this.addPages(this.config.pages);
    }

    if (this._configSetup) {
      this._configSetup.bind(config)();
    }
  };

  this.draw = function() {
    // removes DOM elements that aren't removed by p5's background()
    for (var i = 0; i < this.pages.length; i++) {
      //this.pages[i].hideElements();
    }

    if (this.config.draw) {
      this.config.draw.bind(config)();
    } else {
      this.pages[this.pageCurrent].draw();
    }
  };

  this.getCurrentPage = function() {
    return this.pages[this.pageCurrent];
  }

  this.addPages = function(pages) {
    for (var i = 0; i < pages.length; i++) {
      pages[i].app = this;
      var page = new tr.gui.page(pages[i]);
      this.pages.push(page);
    }
  }

  this._drawIconText = function(iconWidth, iconHeight, iconMargin) {
    fill(255);
    noStroke();
    textAlign(CENTER, BOTTOM);
    textSize(16);
    text(this.name, 0, iconHeight - (iconMargin / 2.0), iconWidth);
  };

  this.drawIcon = function(iconWidth, iconHeight, iconMargin) {
    stroke(0);
    fill(150);
    if (this._configDrawIcon) {
      this._configDrawIcon.bind(config)(iconWidth, iconHeight, iconMargin);
    } else if (this.iconImg) {
      if (!this.iconImgMask) {
        this.iconImgMask = createGraphics(iconWidth, iconHeight);
        this.iconImgMask.rect(0, 0, iconWidth, iconHeight, 15);
        this.iconImgMask.fill('rgba(0,0,0,1)');
      }
      this.iconImg.resize(iconWidth - iconMargin * 3, iconHeight - iconMargin * 3);
      this.iconImg.mask(this.iconImgMask);
      image(this.iconImg, iconMargin * 1.5, iconMargin);
    }

    this._drawIconText(iconWidth, iconHeight, iconMargin);
  };

  this.open = function() {
    appSelected = this.id;
  };

  this.setPage = function(p) {
    this.pages[this.pageCurrent].hideElements();
    this.pageCurrent = p;
  }

  this.close = function() {
    // removes DOM elements that aren't removed by p5's background()
    for (var i = 0; i < this.pages.length; i++) {
      this.pages[i].hideElements();
    }

    this.pageCurrent = 0;
    desktop.open();
  };

  this.mousePressed = function() {
    if (this._configMousePressed) {
      this._configMousePressed.bind(config)();
    }
    if (this.pageCurrent > -1) {
      this.pages[this.pageCurrent].mousePressed();
    }
  };

  this.mouseReleased = function() {
    if (this._configMouseReleased) {
      this._configMouseReleased.bind(config)();
    }
    if (this.pageCurrent > -1) {
      this.pages[this.pageCurrent].mouseReleased();
    }
  };

  this.mouseClicked = function() {
    if (this._configMouseClicked) {
      this._configMouseClicked.bind(config)();
    }
    if (this.pageCurrent > -1) {
      this.pages[this.pageCurrent].mouseClicked();
    }
  };

  this.mouseDragged = function() {
    if (this._configMouseDragged) {
      this._configMouseDragged.bind(config)();
    }
    if (this.pageCurrent > -1) {
      this.pages[this.pageCurrent].mouseDragged();
    }
  }

  this.keyPressed = function() {
    if (this._configKeyPressed) {
      this._configKeyPressed.bind(config)();
    }
  };

}
