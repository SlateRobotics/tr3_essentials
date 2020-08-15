if (!tr) tr = {};
if (!tr.app) tr.app = {};

function App(config) {
  this.id = config.id;
  if (config.id == null) this.id = -1;

  this.name = config.name;
  this.enabled = config.enabled;
  if (config.enabled == null) this.enabled = true;

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

  this.setup = function() {
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
if (!tr) tr = {};
if (!tr.app) tr.app = {};

tr.app.butler = new App({
  id: 0,
  name: "Butler",
  desc: "Schedule Domestic Services and Chores",
  iconUrl: "/img/icon-app-butler",
  state: {

  },
  pages: [{
    id: "main",
    pos: {
      x: 0,
      y: 0
    },
    size: {
      w: 1.0,
      h: 1.0
    },
    header: {
      text: "Butler"
    },
    children: [{
      id: "colLeft",
      type: "container",
      size: {
        w: 0.5,
        h: 1
      },
      background: "rgba(255, 255, 255, 0.2)",
      radius: 15,
      margin: 10,
      padding: 15,
      children: [{
        type: "container",
        size: {
          w: 0.5,
          h: 1
        },
        border: "false",
        children: [{
          type: "text",
          text: "Plan:",
          textSize: 20,
        }, {
          type: "text",
          text: "Time Remaining:",
          textSize: 20,
        }],
      }, {
        type: "container",
        size: {
          w: 0.5,
          h: 1
        },
        border: "false",
        children: [{
          type: "text",
          align: {
            v: "TOP",
            h: "RIGHT"
          },
          text: "Basic",
          textSize: 20,
        }, {
          type: "text",
          align: {
            v: "TOP",
            h: "RIGHT"
          },
          text: "4 hrs 15 min",
          textSize: 20,
        }],
      }],
    }, {
      id: "scheduleservice",
      type: "container",
      size: {
        w: 0.5,
        h: 0.75
      },
      margin: 10,
      padding: 15,
      background: "rgba(37, 183, 250, 0.8)",
      radius: 15,
      align: {
        v: "CENTER",
        h: "CENTER"
      },
      onClick: function() {
        this.getApp().setPage(1);
      },
      children: [{
        type: "container",
        size: {
          w: 1.0,
          h: 60
        },
        align: {
          v: "CENTER",
          h: "CENTER"
        },
        border: false,
        children: [{
          type: "text",
          text: "SCHEDULE",
          textSize: 28,
          size: {
            w: 1,
            h: 30
          },
          align: {
            v: "TOP",
            h: "CENTER"
          },
        }, {
          id: "lblService",
          type: "text",
          text: "SERVICE",
          textSize: 28,
          size: {
            w: 1,
            h: 30
          },
          align: {
            v: "TOP",
            h: "CENTER"
          },
        }],
      }]
    }],
  }, {
    id: "schedule",
    size: {
      w: 1.0,
      h: 1.0
    },
    header: {
      text: "Butler | Schedule Service"
    },
    children: [{
      type: "container",
      size: {
        w: 1,
        h: 0.8
      },
      margin: 10,
      padding: 15,
      background: "rgba(255,255,255,0.2)",
      radius: 15,
      children: [{
        type: "container",
        size: {
          w: 0.5,
          h: 1
        },
        border: false,
        align: {
          v: "TOP",
          h: "LEFT"
        },
        children: [{
          type: "text",
          size: {
            w: 1,
            h: 40
          },
          text: "Date of Service:",
          align: {
            v: "CENTER",
            h: "LEFT"
          },
        }, {
          type: "text",
          size: {
            w: 1,
            h: 40
          },
          text: "Service Type:",
          align: {
            v: "CENTER",
            h: "LEFT"
          },
        }],
      }, {
        type: "container",
        size: {
          w: 0.5,
          h: 1
        },
        border: false,
        children: [{
          type: "datePicker",
          border: false,
          size: {
            w: 1,
            h: 40
          },
          onChange: function(d) {
            console.log(d);
          },
        }, {
          type: "select",
          size: {
            w: 1,
            h: 40
          },
          padding: 5,
          options: ["Laundry", "Kitchen Cleanup", "House Pickup"],
          textSize: 22,
        }],
      }],
    }, {
      id: "buttons",
      type: "container",
      size: {
        w: 1.0,
        h: 0.2
      },
      border: "false",
      children: [{
        type: "container",
        size: {
          w: 0.5,
          h: 1
        },
        border: false,
      }, {
        type: "container",
        size: {
          w: 0.25,
          h: 1
        },
        background: "rgba(255, 255, 255, 0.2)",
        radius: 15,
        margin: 10,
        padding: 15,
        onClick: function() {
          this.getApp().setPage(0);
        },
        children: [{
          type: "text",
          size: {
            w: 1,
            h: 1
          },
          align: {
            v: "CENTER",
            h: "CENTER"
          },
          text: "Close",
          textSize: 22,
        }],
      }, {
        type: "container",
        size: {
          w: 0.25,
          h: 1
        },
        background: "rgba(37, 183, 250, 0.8)",
        radius: 15,
        margin: 10,
        padding: 15,
        onClick: function() {
          this.getApp().setPage(0);
        },
        children: [{
          type: "text",
          size: {
            w: 1,
            h: 1
          },
          align: {
            v: "CENTER",
            h: "CENTER"
          },
          text: "Submit",
          textSize: 22,
        }],
      }],
    }],
  }],
});
if (!tr) tr = {};
if (!tr.app) tr.app = {};

tr.app.chess = new App({
  name: "Chess",
  iconUrl: "/img/icon-app-chess",
  enabled: false,
  pages: [{
    id: "main",
    header: {
      text: "Chess",
    },
    children: [{
      type: "container",
      margin: 10,
      padding: 10,
      background: "rgba(255, 255, 255, 0.2)",
      radius: 15,
      children: [],
    }],
  }],
});
var appCircles = new App({

  ////////////////////////////////////////
  // vvv CONFIG USED BY CONSTRUCTOR vvv //
  ////////////////////////////////////////

  name: "Circles!",
  iconUrl: "https://cdn1.iconfinder.com/data/icons/material-core/20/check-circle-outline-blank-512.png",
  k: 0,
  cx: 0,
  cy: 0,
  cs: 50,
  scx: 0,
  scy: 0,
  tcx: 0,
  tcy: 0,
  fcx: 0,
  fcy: 0,
  mov: 50,
  go: false,
  circles: [],
  button: '',


  setup: function() {
    this.cx = canvasWidth / 2;
    this.cy = canvasHeight / 2;
    this.button = createButton('X');
    this.button.position(825, 10);
    this.button.mousePressed(this.close.bind(this));
    this.button.hide();


  },

  draw: function() {
    background(240);
    fill(0);
    text(this.name + "  Keys Pressed: " + this.k, 100, 50);
    push();
    noFill();
    strokeWeight(4);
    stroke(51);
    circle(this.cx, this.cy, this.cs);
    this.button.show();
    this.overDraw2();
    this.overDraw3();
    this.overDraw4();

    for (let i = 0; i < this.circles.length; i++) {
      this.circles[i].display();
    }


    pop();
    if (this.go == true) {
      if (this.cx < canvasWidth + this.cs) {
        this.cx++;
      } else {
        this.cx = this.cs;
      }
    }
  },

  mousePressed: function() {
    //this._app.close();
    this.circles.push(new ccircles(mouseX, mouseY, this.cs));
  },

  keyPressed: function() {
    console.log(keyCode);
    if (key) {
      if (keyCode == SHIFT) {
        if (this.go == false) {
          this.go = true;
        } else {
          this.go = false;
        }
      } else if (keyCode == UP_ARROW) {
        this.cy = this.cy - this.mov;
        this.k++;
      } else if (keyCode == DOWN_ARROW) {
        this.cy = this.cy + this.mov;
        this.k++;
      } else if (keyCode == LEFT_ARROW) {
        this.cx = this.cx - this.mov;
        this.k++;
      } else if (keyCode == RIGHT_ARROW) {
        this.cx = this.cx + this.mov;
        this.k++;
      } else if (keyCode == ENTER) {
        this.circles.push(new ccircles(this.cx, this.cy, this.cs));
      } else if (keyCode == 8) { //BACKSPACE\\
        this.circles = [];
        this.cx = 300;
        this.cy = 300;
        this.k = 0;
      } else if (keyCode == 27) { //ESC\\
        this._app.close();
      } else {}
    }
  },


  ////////////////////////////////////////
  // vvv USER DEFINED / HELPER DATA vvv //
  ////////////////////////////////////////

  close: function() {
    this.button.hide();
    this._app.close();

  },



  overDraw2: function() {
    if (this.cx > canvasWidth - this.cs) {
      this.scx = this.cx - canvasWidth;
      if (this.cy > canvasHeight - this.cs) {
        this.scy = this.cy - canvasHeight;
      } else if (this.cy < 0 + this.cs) {
        this.scy = this.cy + canvasHeight;
      } else {
        this.scy = this.cy;
      }
    }
    if (this.cy > canvasHeight - this.cs) {
      this.scy = this.cy - canvasHeight;
      if (this.cx > canvasWidth - this.cs) {
        this.scx = this.cx - canvasWidth;
      } else if (this.cx < 0 + this.cs) {
        this.scx = this.cx + canvasWidth;
      } else {
        this.scx = this.cx;
      }
    }
    if (this.cx < 0 + this.cs) {
      this.scx = this.cx + canvasWidth;
      if (this.cy > canvasHeight - this.cs) {
        this.scy = this.cy - canvasHeight;
      } else if (this.cy < 0 + this.cs) {
        this.scy = this.cy + canvasHeight;
      } else {
        this.scy = this.cy;
      }
    }
    if (this.cy < 0 + this.cs) {
      this.scy = this.cy + canvasHeight;
      if (this.cx > canvasWidth - this.cs) {
        this.scx = this.cx - canvasWidth;
      } else if (this.cx < 0 + this.cs) {
        this.scx = this.cx + canvasWidth;
      } else {
        this.scx = this.cx;
      }
    }
    if (this.cy < 0 + this.cs || this.cx < 0 + this.cs || this.cy > canvasHeight - this.cs || this.cx > canvasWidth - this.cs) {
      circle(this.scx, this.scy, this.cs);
    }
    if (this.scy > this.cs && this.scy < canvasHeight - this.cs) {
      this.cy = this.scy;
      this.scy = 0;
    }
    if (this.scx > this.cs && this.scx < canvasWidth - this.cs) {
      this.cx = this.scx;
      this.scx = 0;
    }

  },
  overDraw3: function() {
    if (this.cx > canvasWidth - this.cs && this.cy > canvasHeight - this.cs || this.cx > canvasWidth - this.cs && this.cy < this.cs) {
      this.tcx = this.cx - canvasWidth;
      this.tcy = this.cy;
      circle(this.tcx, this.tcy, this.cs);
    }
    if (this.cx < this.cs && this.cy < this.cs || this.cx < this.cs && this.cy > canvasHeight - this.cs) {
      this.tcx = this.cx + canvasWidth;
      this.tcy = this.cy;
      circle(this.tcx, this.tcy, this.cs);
    }

  },
  overDraw4: function() {
    if (this.cx > canvasWidth - this.cs && this.cy > canvasHeight - this.cs || this.cx < this.cs && this.cy > canvasHeight - this.cs) {
      this.fcx = this.cx;
      this.fcy = this.cy - canvasHeight;
      circle(this.fcx, this.fcy, this.cs);
    }
    if (this.cx < this.cs && this.cy < this.cs || this.cx > canvasWidth - this.cs && this.cy < this.cs) {
      this.fcx = this.cx;
      this.fcy = this.cy + canvasHeight;
      circle(this.fcx, this.fcy, this.cs);
    }
  },

});
var ccircles = function(nx, ny, ns) {
  this.x = nx;
  this.y = ny;
  this.s = ns;
  this.display = function() {
    circle(this.x, this.y, this.s);
  };

};
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.btnAdd = function(rostopic, value) {
  return {
    type: "container",
    size: {
      w: 1 / 4,
      h: 50
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        if (rostopic && value) {
          tr.data.socket.emit(rostopic, value);
        }
      },
      children: [{
        type: "text",
        text: "+",
        textSize: 24,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.btnBig = function(text, rostopic, value, background) {
  return {
    type: "container",
    size: {
      w: 0.25,
      h: 50
    },
    background: background,
    onClick: function() {
      tr.data.socket.emit(rostopic, value);
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: text,
        textSize: 18,
        padding: 12,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.btnCalibrate = function(id) {
  return {
    type: "container",
    background: "rgb(150, 150, 150)",
    size: {
      w: 0.111,
      h: 20
    },
    onClick: function() {
      tr.data.socket.emit("/tr3/joints/" + id + "/calibrate", true);
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: "Calibrate",
        textSize: 14,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.btnMotorDir = function(id) {
  return {
    type: "container",
    background: "rgb(150, 150, 150)",
    size: {
      w: 0.111,
      h: 20
    },
    onClick: function() {
      tr.data.socket.emit("/tr3/joints/" + id + "/flip", true);
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: "Flip",
        textSize: 14,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.btnPID = function(id) {
  return {
    type: "container",
    background: "rgb(150, 150, 150)",
    size: {
      w: 1 / 18,
      h: 20
    },
    onClick: function() {
      var app = this.getApp();
      var page = app.getCurrentPage();

      var p = page.getChild(id + "slider-p").element.value();
      var i = page.getChild(id + "slider-i").element.value();
      var d = page.getChild(id + "slider-d").element.value();

      tr.data.socket.emit("/tr3/joints/" + id + "/pid", [p, i, d]);
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: "->",
        textSize: 14,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.btnPause = function(rostopic, value) {
  return {
    type: "container",
    size: {
      w: 1 / 4,
      h: 50
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        if (rostopic && value) {
          tr.data.socket.emit(rostopic, value);
        }
      },
      children: [{
        type: "text",
        text: "II", // ▶ ◀
        textSize: 24,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.btnPlay = function(rostopic, value) {
  return {
    type: "container",
    size: {
      w: 1 / 4,
      h: 50
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        if (rostopic && value) {
          tr.data.socket.emit(rostopic, value);
        }
      },
      children: [{
        type: "text",
        text: "▶", // ▶ ◀
        textSize: 16,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.btnRemove = function(rostopic, value) {
  return {
    type: "container",
    size: {
      w: 1 / 4,
      h: 50
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        if (rostopic && value) {
          tr.data.socket.emit(rostopic, value);
        }
      },
      children: [{
        type: "text",
        text: "−",
        textSize: 24,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.btnResetPos = function(id) {
  return {
    type: "container",
    background: "rgb(150, 150, 150)",
    size: {
      w: 0.111,
      h: 20
    },
    onClick: function() {
      tr.data.socket.emit("/tr3/joints/" + id + "/reset", true);
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: "Reset",
        textSize: 14,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.btnToglStop = function() {
  var currentmode = "STOPPED"
  return {
    type: "container",
    size: {
      w: 3 / 9,
      h: 120
    },
    background: "red",
    onClick: function() {
      if (currentmode == "STOPPED") {
        currentmode = "RELEASED"
        this.background = "green"
        var app = this.getApp();
        var page = app.getCurrentPage();
        page.getChild("ToglStop").text = "RELEASE";
        tr.data.socket.emit("/tr3/stop", true, );
      } else if (currentmode == "RELEASED") {
        currentmode = "STOPPED"
        this.background = "red"
        var app = this.getApp();
        var page = app.getCurrentPage();
        page.getChild("ToglStop").text = "STOP";
        tr.data.socket.emit("/tr3/stop", false, );
      }

    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: "ToglStop",
        type: "text",
        text: "STOP",
        textSize: 52,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.btnWaypointLeft = function(rostopic, value) {
  return {
    type: "container",
    size: {
      w: 1 / 4,
      h: 50
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        if (rostopic && value) {
          tr.data.socket.emit(rostopic, value);
        }
      },
      children: [{
        type: "text",
        text: "〈 ", // ▶ ◀
        textSize: 16,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.btnWaypointRight = function(rostopic, value) {
  return {
    type: "container",
    size: {
      w: 1 / 4,
      h: 50
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        if (rostopic && value) {
          tr.data.socket.emit(rostopic, value);
        }
      },
      children: [{
        type: "text",
        text: " 〉", // ▶ ◀
        textSize: 16,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.configHeader = function() {
  var c = tr.controls.controlPanel;
  return [c.label("IDs"), c.label("Motor Dir"), c.label("Reset Pos"), c.label("Calibrate"), c.label("PID Tuning", 5 / 9)];
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.configRow = function(id) {
  var c = tr.controls.controlPanel;
  return [c.labelID(id), c.btnMotorDir(id), c.btnResetPos(id), c.btnCalibrate(id), c.txtP(id), c.sliderP(id), c.txtI(id), c.sliderI(id), c.txtD(id), c.sliderD(id), c.btnPID(id)];
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.controlHeader = function() {
  var c = tr.controls.controlPanel;
  return [c.label("IDs"), c.label("Position"), c.label("Mode"), c.label("Target"), c.label("Position Slider", 5 / 9)];
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.controlRow = function(id) {
  var c = tr.controls.controlPanel;
  return [c.labelID(id), c.txtState(id), c.selectMode(id), c.txtTarget(id), c.slider(id)];
}
if (!tr) tr = {};
if (!tr.app) tr.app = {};

tr.app.controlPanel = function() {
  var c = tr.controls.controlPanel;

  return new App({
    id: 1,
    name: "Control Panel",
    iconUrl: "/img/icon-app-control",
    pages: [{
      pos: {
        x: 0,
        y: 0
      },
      size: {
        w: 1.0,
        h: 1.0
      },
      header: {
        text: "Control Panel",
      },
      children: [{
        type: "tabControl",
        labels: ["Control", "Config", "Camera", "3D Render"],
        pages: [c.tabControl(), c.tabConfig(), c.tabCamera(), c.tabRender()],
      }],
    }],
  });
};
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.label = function(id, w) {
  if (!w) w = 1.0 / 9.0;
  return {
    type: "container",
    size: {
      w: w,
      h: 20
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: id,
        textSize: 16,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.labelID = function(id, w) {
  if (!w) w = 1.0 / 9.0;
  return {
    type: "container",
    size: {
      w: w,
      h: 20
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: id,
        textSize: 14,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.labelProgram = function() {
  return {
    type: "container",
    size: {
      w: 1 / 4,
      h: 50
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: "program-label",
        type: "text",
        text: "P#",
        textSize: 16,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.playbackBlock = function() {
  var c = tr.controls.controlPanel;

  var children = [];
  children.push(c.btnPlay());
  children.push(c.btnPause());
  children.push(c.btnAdd());
  children.push(c.btnRemove());
  children.push(c.btnWaypointLeft());
  children.push(c.labelProgram());
  children.push(c.btnWaypointRight());
  //children.push(c.btnPlaybackImg("/img/pnp-control0.png"));

  return {
    type: "container",
    size: {
      w: 3 / 9,
      h: 120,
    },
    padding: 10,
    background: "rgba(255, 255, 255, 0.2)",
    children: children,
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.playbackDisplay = function() {
  var c = tr.controls.controlPanel;

  var children = [];
  children.push(c.playbackLabel("a0"));
  children.push(c.playbackValue("a0"));
  children.push(c.playbackLabel("a4"));
  children.push(c.playbackValue("a4"));
  children.push(c.playbackLabel("a1"));
  children.push(c.playbackValue("a1"));
  children.push(c.playbackLabel("g0"));
  children.push(c.playbackValue("g0"));
  children.push(c.playbackLabel("a2"));
  children.push(c.playbackValue("a2"));
  children.push(c.playbackLabel("h0"));
  children.push(c.playbackValue("h0"));
  children.push(c.playbackLabel("a3"));
  children.push(c.playbackValue("a3"));
  children.push(c.playbackLabel("h1"));
  children.push(c.playbackValue("h1"));

  return {
    type: "container",
    size: {
      w: 3 / 9,
      h: 120,
    },
    padding: 10,
    background: "rgba(255, 255, 255, 0.2)",
    children: children,
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.playbackLabel = function(id) {
  return {
    type: "container",
    size: {
      w: 1 / 4,
      h: 1 / 4
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: id,
        textSize: 14,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.playbackValue = function(id) {
  return {
    type: "container",
    size: {
      w: 1 / 4,
      h: 1 / 4
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: "0.00",
        textSize: 14,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
        onDraw: function() {
          // Replace with Waypoint Controll Stuff
          var p = tr.data.getState(id).position;
          if (p) {
            this.text = p.toFixed(2);
          }
        }
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.selectMode = function(id) {
  return {
    type: "container",
    size: {
      w: 1 / 9,
      h: 20
    },

    children: [{
      type: "container",
      border: false,
      children: [{
        type: "select",
        id: "select-" + id,
        options: ["EFFORT", "BACKDRIVE", "SERVO"],
        onChange: function(val) {
          var app = this.getApp();
          var page = app.getCurrentPage();
          if (page) {
            var slider = page.getChild(id + "slider");
            if (slider) {
              if (val == "EFFORT") {
                slider.setval(0);
              } else if (val == "BACKDRIVE") {
                slider.setval(0);
              } else if (val == "SERVO") {
                var state = tr.data.getState(id).position;
                slider.setval(state / Math.PI / 2.0);
              }
            }
          }

          var i = 0;
          if (val == "EFFORT") {
            i = 0;
          } else if (val == "BACKDRIVE") {
            i = 1;
          } else if (val == "SERVO") {
            i = 2;
          }

          tr.data.socket.emit("/tr3/joints/" + id + "/mode", i);
        },
        Size: {
          w: 1,
          h: 20
        },
        textSize: 12,
        padding: 0,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.slider = function(id) {
  return {
    type: "container",
    size: {
      w: 5 / 9,
      h: 20
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: id + "slider",
        type: "slider",

        onDraw: function() {
          var app = this.getApp();
          var page = app.getCurrentPage();
          var select = page.getChild("select-" + id);
          var sliderVal = page.getChild(id + "slider").element.value();

          if (select.element.value() == "EFFORT" && abs(sliderVal) > 0.1) {
            tr.data.socket.emit("/tr3/joints/" + id + "/control/effort", sliderVal);
          }
        },

        onInput: function(val) {
          var app = this.getApp();
          var page = app.getCurrentPage();
          var select = page.getChild("select-" + id);
          var sliderVal = page.getChild(id + "slider").element.value();

          var val = 0;
          if (select.element.value() == "EFFORT") {
            val = sliderVal;
            tr.data.socket.emit("/tr3/joints/" + id + "/control/effort", val);
          } else if (select.element.value() == "SERVO") {
            val = sliderVal * Math.PI * 2.0;
            tr.data.socket.emit("/tr3/joints/" + id + "/control/position", val);
          }

          var label = page.getChild(id + "sliderl");
          label.text = val.toFixed(2);
        },

        onChange: function(val) {
          var app = this.getApp();
          var page = app.getCurrentPage();
          var select = page.getChild("select-" + id);
          if (select.element.value() == "EFFORT") {
            var val = 0;

            var slider = page.getChild(id + "slider");
            slider.element.value(val);
            tr.data.socket.emit("/tr3/joints/" + id + "/control/effort", val);

            var label = page.getChild(id + "sliderl");
            label.text = val.toFixed(2);
          }
        },

        min: -1,
        max: 1,
        val: 0,
        step: 0.01,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.sliderD = function(id) {
  return {
    type: "container",
    size: {
      w: 1 / 9,
      h: 20
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: id + "slider-d",
        type: "slider",

        onInput: function(val) {
          var app = this.getApp();
          var page = app.getCurrentPage();
          var select = page.getChild("select-" + id);
          var sliderVal = page.getChild(id + "slider-d").element.value();

          var label = page.getChild("txt-" + id + "-d");
          label.text = val.toFixed(1);
        },

        onChange: function(val) {
          var app = this.getApp();
          var page = app.getCurrentPage();
          var select = page.getChild("select-" + id);

          var slider = page.getChild(id + "slider-d");
          var val = slider.element.value();

          var label = page.getChild("txt-" + id + "-d");
          label.text = val.toFixed(1);
        },

        min: 0,
        max: 2,
        val: 0.2,
        step: 0.1,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.sliderI = function(id) {
  return {
    type: "container",
    size: {
      w: 1 / 9,
      h: 20
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: id + "slider-i",
        type: "slider",

        onInput: function(val) {
          var app = this.getApp();
          var page = app.getCurrentPage();
          var select = page.getChild("select-" + id);
          var sliderVal = page.getChild(id + "slider-i").element.value();

          var label = page.getChild("txt-" + id + "-i");
          label.text = val.toFixed(1);
        },

        onChange: function(val) {
          var app = this.getApp();
          var page = app.getCurrentPage();
          var select = page.getChild("select-" + id);

          var slider = page.getChild(id + "slider-i");
          var val = slider.element.value();

          var label = page.getChild("txt-" + id + "-i");
          label.text = val.toFixed(1);
        },

        min: 0,
        max: 20,
        val: 5,
        step: 0.1,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.sliderP = function(id) {
  return {
    type: "container",
    size: {
      w: 1 / 9,
      h: 20
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: id + "slider-p",
        type: "slider",

        onInput: function(val) {
          var app = this.getApp();
          var page = app.getCurrentPage();
          var select = page.getChild("select-" + id);
          var sliderVal = page.getChild(id + "slider-p").element.value();

          var label = page.getChild("txt-" + id + "-p");
          label.text = val.toFixed(1);
        },

        onChange: function(val) {
          var app = this.getApp();
          var page = app.getCurrentPage();
          var select = page.getChild("select-" + id);

          var slider = page.getChild(id + "slider-p");
          var val = slider.element.value();

          var label = page.getChild("txt-" + id + "-p");
          label.text = val.toFixed(1);
        },

        min: 0,
        max: 20,
        val: 9,
        step: 0.1,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.spacer = function() {
  var c = tr.controls.controlPanel;

  return {
    type: "container",
    size: {
      w: 1.0,
      h: 10
    },
    border: false
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.tabCamera = function() {
  var c = tr.controls.controlPanel;

  return {
    type: "container",
    size: {
      w: 1.0,
      h: "fill"
    },
    background: "rgba(255, 255, 255, 0.2)",
    children: [{
      type: "camera"
    }],
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.tabConfig = function() {
  var c = tr.controls.controlPanel;

  var children = [];
  children.push.apply(children, c.configHeader());
  children.push.apply(children, c.configRow("a0"));
  children.push.apply(children, c.configRow("a1"));
  children.push.apply(children, c.configRow("a2"));
  children.push.apply(children, c.configRow("a3"));
  children.push.apply(children, c.configRow("a4"));
  children.push.apply(children, c.configRow("g0"));
  children.push.apply(children, c.configRow("h0"));
  children.push.apply(children, c.configRow("h1"));
  children.push.apply(children, c.configRow("b0"));
  children.push.apply(children, c.configRow("b1"));

  return {
    type: "container",
    size: {
      w: 1.0,
      h: "fill"
    },
    padding: 10,
    background: "rgba(255, 255, 255, 0.2)",
    children: children,
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.tabControl = function() {
  var c = tr.controls.controlPanel;

  var children = [];
  children.push.apply(children, c.controlHeader());
  children.push.apply(children, c.controlRow("a0"));
  children.push.apply(children, c.controlRow("a1"));
  children.push.apply(children, c.controlRow("a2"));
  children.push.apply(children, c.controlRow("a3"));
  children.push.apply(children, c.controlRow("a4"));
  children.push.apply(children, c.controlRow("g0"));
  children.push.apply(children, c.controlRow("h0"));
  children.push.apply(children, c.controlRow("h1"));

  children.push(c.spacer());
  children.push(c.btnToglStop());
  children.push(c.playbackBlock());
  children.push(c.playbackDisplay());

  return {
    type: "container",
    size: {
      w: 1.0,
      h: "fill"
    },
    padding: 10,
    background: "rgba(255, 255, 255, 0.2)",
    children: children,
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.tabRender = function() {
  var c = tr.controls.controlPanel;

  return {
    type: "container",
    size: {
      w: 1.0,
      h: "fill"
    },
    padding: 10,
    background: "rgb(100, 100, 100)",
    children: [{
      type: "container",
      border: false,
      size: {
        w: 1.0,
        h: "fill"
      },
      padding: 0,
      children: [{
        type: "tr2",
      }]
    }],
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.txtD = function(id) {
  return {
    type: "container",
    size: {
      w: 1 / 18,
      h: 20
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: "txt-" + id + "-d",
        type: "text",
        text: "0.2",
        textSize: 14,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.txtI = function(id) {
  return {
    type: "container",
    size: {
      w: 1 / 18,
      h: 20
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: "txt-" + id + "-i",
        type: "text",
        text: "5.0",
        textSize: 14,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.txtP = function(id) {
  return {
    type: "container",
    size: {
      w: 1 / 18,
      h: 20
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: "txt-" + id + "-p",
        type: "text",
        text: "9.0",
        textSize: 14,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.txtState = function(id) {
  return {
    type: "container",
    size: {
      w: 0.111,
      h: 20
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: "",
        textSize: 14,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
        onDraw: function() {
          var p = tr.data.getState(id).position;
          if (p) {
            this.text = p.toFixed(2);
          }
        }
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.txtTarget = function(id) {
  return {
    type: "container",
    size: {
      w: 0.111,
      h: 20
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: id + "sliderl",
        type: "text",
        text: "0.00",
        textSize: 14,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
var desktop = {
  id: -1,

  apps: [],

  imageBackground: "",

  menuRows: 3,
  menuCols: 3,
  menuWidth: 400,
  menuHeight: 400,
  menuMargin: 20,
  menuX: (864 - 400) - 20,
  menuY: ((480 - 400) / 2),

  menuTransX: 0,
  menuTransXTarget: 0,
  menuTransXStart: 0,
  menuTransXTargetStep: 0,
  offsetX: 0,

  dragging: false,
  draggingStarted: 0,

  setup: function(id) {
    this.id = id;
    this.imageBackground = loadImage('/img/slate-background');
  },


  draw: function() {
    background('rgb(34,34,34)');
    this.imageBackground.resize(canvasWidth, canvasHeight);
    image(this.imageBackground, 0, 0);

    this.computeDrag();
    this.drawInfoScreen();
    this.drawAppMenu();
  },

  drawInfoScreen: function() {
    fill(255);
    noStroke();
    textAlign(LEFT, TOP);
    textSize(36);

    var t = moment().format("h:mm A");
    text(t, this.menuMargin, this.menuY + this.menuMargin);

    textSize(24);
    var d = moment().format("dddd MMM Do");
    text(d, this.menuMargin, this.menuY + this.menuMargin + 40);
  },

  drawAppMenu: function() {
    translate(this.menuX, this.menuY);
    for (var i = 0; i < this.menuCols; i++) {
      for (var j = 0; j < this.menuRows; j++) {
        var appIndex = j + i * this.menuRows;
        if (appIndex >= this.apps.length) {
          break;
        }

        var pos = this.computeIconPose(i, j);
        translate(pos.x, pos.y);
        this.apps[appIndex].drawIcon(pos.width, pos.height, this.menuMargin);
        translate(-pos.x, -pos.y);
      }
    }
    translate(-this.menuX, -this.menuY);
  },

  computeDrag: function() {
    if (this.dragging) {
      this.menuTransX = mouseX + this.offsetX;
    }

    if (this.menuTransX < this.menuTransXTarget - 1) {
      this.menuTransX += this.menuTransXTargetStep;
    } else if (this.menuTransX > this.menuTransXTarget + 1) {
      this.menuTransX -= this.menuTransXTargetStep;
    } else {
      this.menuTransX = this.menuTransXTarget
    }

    translate(this.menuTransX, 0);
  },

  computeIconPose: function(i, j) {
    var itemX = j * this.menuWidth / this.menuCols;
    var itemY = i * this.menuHeight / this.menuRows;
    return {
      x: itemX,
      y: itemY,
      width: this.menuWidth / this.menuCols,
      height: this.menuHeight / this.menuRows,
    };
  },

  open: function() {
    appSelected = this.id;
  },

  mousePressed: function() {
    this.dragging = true;
    this.draggingStarted = +new Date();
    this.offsetX = this.menuTransX - mouseX;
    this.menuTransXStart = this.menuTransX;
  },

  mouseReleased: function() {
    this.dragging = false;

    var draggingEnded = +new Date();
    var timeDiff = draggingEnded - this.draggingStarted;
    if (timeDiff < 250) {
      this.appClick();
    }

    var m = this.menuTransX % canvasWidth;
    var p = -m / canvasWidth;
    var n = 0;

    if (p > 0.25 && -this.menuTransX - -this.menuTransXStart > 0) {
      n = 0;
    } else if (p > 0.25 && p < 0.75 && -this.menuTransX - -this.menuTransXStart <= 0) {
      n = -1;
    } else if (p < 0.25) {
      n = -1;
    }

    var pageNum = floor(this.menuTransX / canvasWidth) - n;
    if (pageNum > 0) {
      pageNum = 0;
    } else if (pageNum < -floor((this.apps.length - 1) / (this.menuRows * this.menuCols))) {
      pageNum = -floor((this.apps.length - 1) / (this.menuRows * this.menuCols));
    }

    this.menuTransXTarget = pageNum * canvasWidth;
    this.menuTransXTargetStep = abs(this.menuTransXTarget - this.menuTransX) / 10.0;
  },

  mouseClicked: function() {},

  mouseDragged: function() {},

  keyPressed: function() {

  },

  appClick: function() {
    for (var i = 0; i < this.menuCols; i++) {
      for (var j = 0; j < this.menuRows; j++) {
        var appIndex = j + i * this.menuRows;
        if (appIndex >= this.apps.length) {
          break;
        }

        var mX = mouseX - this.menuX;
        var mY = mouseY - this.menuY;
        var pos = this.computeIconPose(i, j);
        if (mX > pos.x && mX < pos.x + pos.width && mY > pos.y && mY < pos.y + pos.height) {
          appSelected = this.apps[appIndex].id;
        }
      }
    }
  },
}
var appFace = new App({

  ////////////////////////////////////////
  // vvv CONFIG USED BY CONSTRUCTOR vvv //
  ////////////////////////////////////////

  name: "Face App",

  draw: function() {
    background(240);
    this._drawLeftEye();
    this._drawRightEye();
  },

  drawIcon: function(iconWidth, iconHeight, iconMargin) {
    stroke(0);
    fill(255);
    var eW = 30;
    var eH = iconHeight - iconMargin * 2 - 25;
    rect(iconMargin, iconMargin, eW, eH);
    rect(iconWidth - eW - iconMargin, iconMargin, eW, eH);

    fill(0);
    rect((eW / 2) + (iconMargin / 2), eH - iconMargin, 20, 20);
    rect(88, eH - iconMargin, 20, 20);
  },

  mousePressed: function() {
    this._app.close();
  },

  ////////////////////////////////////////
  // vvv USER DEFINED / HELPER DATA vvv //
  ////////////////////////////////////////

  eyeWidth: 150,
  eyeHeight: 250,
  eyeDistance: 200,

  _drawLeftEye: function() {
    stroke(0);
    fill(255);
    var x = (canvasWidth / 2) - this.eyeWidth - (this.eyeDistance / 2);
    var y = (canvasHeight / 2) - (this.eyeHeight / 2) - 50;
    rect(x, y, this.eyeWidth, this.eyeHeight);

    fill(0);
    var x = x + 50 + (mouseX - canvasWidth / 2) / 20;
    var y = y + this.eyeHeight - 100 + (mouseY - canvasHeight / 2) / 20;
    rect(x, y, 50, 50);
  },

  _drawRightEye: function() {
    var x = (canvasWidth / 2) + (this.eyeDistance / 2);
    var y = (canvasHeight / 2) - (this.eyeHeight / 2) - 50;

    fill(255);
    rect(x, y, this.eyeWidth, this.eyeHeight);

    fill(0);
    var x = x + 50 + (mouseX - canvasWidth / 2) / 20;
    var y = y + this.eyeHeight - 100 + (mouseY - canvasHeight / 2) / 20;
    rect(x, y, 50, 50);
  },
});
if (!tr) tr = {};
if (!tr.app) tr.app = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.frv) tr.controls.frv = {};
tr.controls.frv.btnArm = function() {
  var c = tr.controls.frv;

  return {
    type: "container",
    size: {
      w: 0.333,
      h: 50,
    },
    background: "rgba(255, 255, 255, 0.2)",
    children: [{
      type: "text",
      text: "x",
      align: { v: "CENTER", h: "CENTER" },
      size: { w: 1, h: 1 },
    }]
  }
}
tr.controls.frv.btnBase = function(lbl) {
  var c = tr.controls.frv;

  return {
    type: "container",
    size: {
      w: 0.333,
      h: 50,
    },
    background: "rgb(80, 80, 80)",
    onMousePress: function () {
      var msg = { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 }}
      if (lbl == "▲") {
        msg.linear.x = 10;
      } else if (lbl == "▼") {
        msg.linear.x = -10;
      } else if (lbl == "▶") {
        msg.angular.z = 10;
      } else if (lbl == "◀") {
        msg.angular.z = -10;
      } else if (lbl == "◤") {
        msg.linear.x = 10;
        msg.angular.z = -10;
      } else if (lbl == "◥") {
        msg.linear.x = 10;
        msg.angular.z = 10;
      } else if (lbl == "◣") {
        msg.linear.x = -10;
        msg.angular.z = -10;
      } else if (lbl == "◢") {
        msg.linear.x = -10;
        msg.angular.z = 10;
      }
      tr.data.socket.emit("/tr3/base/diff/cmd_vel", msg);
    },
    onMouseRelease: function () {
      var msg = { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 }}
      tr.data.socket.emit("/tr3/base/diff/cmd_vel", msg);
    },
    onClick: function () {
      if (lbl == "▲") {

      } else if (lbl == "▼") {

      } else if (lbl == "▶") {

      } else if (lbl == "◀") {

      }
    },
    children: [{
      type: "text",
      textFont: "noto",
      text: lbl,
      align: { v: "CENTER", h: "CENTER" },
      size: { w: 1, h: 1 },
    }]
  }
}
tr.controls.frv.btnHead = function(lbl) {
  var c = tr.controls.frv;

  return {
    type: "container",
    size: {
      w: 0.333,
      h: 50,
    },
    background: "rgb(80, 80, 80)",
    onClick: function () {
      var h0 = tr.data.getState("h0").position;
      var h1 = tr.data.getState("h1").position;

      if (lbl == "▲") {
        tr.data.socket.emit("/tr3/joints/h1/control/position", h1 + 0.1);
      } else if (lbl == "▼") {
        tr.data.socket.emit("/tr3/joints/h1/control/position", h1 - 0.1);
      } else if (lbl == "▶") {
        tr.data.socket.emit("/tr3/joints/h0/control/position", h0 - 0.1);
      } else if (lbl == "◀") {
        tr.data.socket.emit("/tr3/joints/h0/control/position", h0 + 0.1);
      } else if (lbl == "◤") {
        tr.data.socket.emit("/tr3/joints/h0/control/position", h0 + 0.1);
        tr.data.socket.emit("/tr3/joints/h1/control/position", h1 + 0.1);
      } else if (lbl == "◥") {
        tr.data.socket.emit("/tr3/joints/h0/control/position", h0 - 0.1);
        tr.data.socket.emit("/tr3/joints/h1/control/position", h1 + 0.1);
      } else if (lbl == "◣") {
        tr.data.socket.emit("/tr3/joints/h0/control/position", h0 + 0.1);
        tr.data.socket.emit("/tr3/joints/h1/control/position", h1 - 0.1);
      } else if (lbl == "◢") {
        tr.data.socket.emit("/tr3/joints/h0/control/position", h0 - 0.1);
        tr.data.socket.emit("/tr3/joints/h1/control/position", h1 - 0.1);
      } else if (lbl = "●") {
        tr.data.socket.emit("/tr3/joints/h0/control/position", 0);
        tr.data.socket.emit("/tr3/joints/h1/control/position", 0);
      }
    },
    children: [{
      type: "text",
      textFont: "noto",
      text: lbl,
      align: { v: "CENTER", h: "CENTER" },
      size: { w: 1, h: 1 },
    }]
  }
}
tr.app.frv = function() {
  var c = tr.controls.frv;

  return new App({
    id: 3,
    name: "Teleop",
    iconUrl: "/img/icon-app-frv",
    pages: [{
      pos: {
        x: 0,
        y: 0
      },
      size: {
        w: 1.0,
        h: 1.0
      },
      header: {
        text: "Teleoperation",
      },
      children: [{
        type: "container",
        size: {
          w: 0.75,
          h: 1.0,
        },
        padding: 0,
        margin: 0,
        background: "rgb(34, 34, 34)",
        children: [{
          type: "tabControl",
          padding: 0,
          margin: 0,
          size: {
            w: 1.0,
            h: "fill",
          },
          labels: ["First-Person", "Third-Person"],
          pages: [{
            type: "container",
            size: {
              w: 1.0,
              h: "fill",
            },
            background: "rgb(34, 34, 34)",
            children: [{
              type: "camera"
            }]
          }, {
            type: "container",
            size: {
              w: 1.0,
              h: "fill",
            },
            background: "rgb(100, 100, 100)",
            children: [{
              type: "tr2"
            }]
          }],
        }],
      }, {
        type: "container",
        size: {
          w: 0.25,
          h: 1.0,
        },
        padding: 0,
        margin: 0,
        background: "rgb(34, 34, 34)",
        children: [{
          type: "container",
          size: {
            w: 1.0,
            h: 0.4,
          },
          background: "rgb(34, 34, 34)",
          children: [{
            type: "minimap",
            size: {
              w: 1,
              h: 1
            }
          }]
        }, {
          type: "container",
          size: {
            w: 1.0,
            h: 0.6,
          },
          padding: 0,
          margin: 0,
          background: "rgb(34, 34, 34)",
          children: [{
            type: "tabControl",
            padding: 0,
            margin: 0,
            labels: ["Base", "Arm", "Head"],
            pages: [c.tabBase(), c.tabArm(), c.tabHead()],
          }]
        }]
      }],
    }],
  });
};
tr.controls.frv.tabArm = function() {
  var c = tr.controls.frv;

  return {
    type: "container",
    size: {
      w: 1.0,
      h: "fill"
    },
    background: "rgb(80, 80, 80)",
    children: [{
      type: "container",
      size: {
        w: 1.0,
        h: "fill",
      },
      children: []
    }],
  }
}
tr.controls.frv.tabBase = function() {
  var c = tr.controls.frv;

  return {
    type: "container",
    size: {
      w: 1.0,
      h: "fill"
    },
    background: "rgb(80, 80, 80)",
    children: [
      c.btnBase("◤"),c.btnBase("▲"),c.btnBase("◥"),
      c.btnBase("◀"),c.btnBase("●"),c.btnBase("▶"),
      c.btnBase("◣"),c.btnBase("▼"),c.btnBase("◢"),
    ],
  }
}
tr.controls.frv.tabHead = function() {
  var c = tr.controls.frv;

  return {
    type: "container",
    size: {
      w: 1.0,
      h: "fill"
    },
    background: "rgb(50, 50, 50)",
    children: [
      c.btnHead("◤"),c.btnHead("▲"),c.btnHead("◥"),
      c.btnHead("◀"),c.btnHead("●"),c.btnHead("▶"),
      c.btnHead("◣"),c.btnHead("▼"),c.btnHead("◢"),
    ],
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.btnRealtime = function() {
  return {
    type: "container",
    size: {
      w: 0.5,
      h: 30
    },
    background: "rgba(255, 255, 255, 0.2)",
    onClick: function() {
      //tr.data.socket.emit(rostopic, value);
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: "Realtime",
        textSize: 18,
        padding: 12,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.btnSimulated = function() {
  return {
    type: "container",
    size: {
      w: 0.5,
      h: 30
    },
    background: "rgba(255, 255, 255, 0.1)",
    onClick: function() {
      //tr.data.socket.emit(rostopic, value);
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: "Simulated",
        textSize: 18,
        padding: 12,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.columnLeft = function() {
  var c = tr.controls.pnp2;

  var children = [];

  children.push(c.programHeader());
  children.push(c.btnSimulated());
  children.push(c.btnRealtime());
  //children.push(c.joystick()); //For Testing - Position values arnt as expected, manualy tuned for this spot.
  children.push(c.tabControl());
  return {
    type: "container",
    size: {
      w: 0.5,
      h: 1,
    },
    background: "rgba(255, 255, 255, 0.2)",
    margin: 10,
    padding: 5,
    children: [{
      type: "container",
      border: false,
      children: children,
}]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.columnRight = function() {
    var c = tr.controls.pnp2;

    var children = [];
    children.push(c.render());
    children.push(c.programBtn_Play());
    children.push(c.programBtn_Pause());
    children.push(c.programBtn_Stop());

  return {
    type: "container",
    size: {
      w: 0.5,
      h: 1,
    },
    background: "rgba(255, 255, 255, 0.2)",
    margin: 10,
    padding: 5,
    children: children,
  }
}
if (!tr) tr = {};
if (!tr.app) tr.app = {};

tr.app.pnp2 = function() {
  var c = tr.controls.pnp2;

var columns = [];
columns.push(c.columnLeft());
columns.push(c.columnRight());

  return new App({
    id: 4,
    name: "P.N.P. V2",
    iconUrl: "/img/icon-app-pnp",
    pages: [{
      pos: {
        x: 0,
        y: 0
      },
      header: {
        text: "P.N.P v2",
      },
      children: columns,
  }],
  });
};
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.inverseBtn_Blnk = function() {
  return {
    type: "container",
    size: {
      w: 1 / 5,
      h: 35
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        //Do waypoint Add
          //tr.data.socket.emit(rostopic, value);
      }
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.inverseBtn_moveDown = function() {
  return {
    type: "container",
    size: {
      w: 1/5,
      h: 35
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "▼", // Add symbol
        textSize: 12,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.inverseBtn_moveLeft = function() {
  return {
    type: "container",
    size: {
      w: 1/5,
      h: 35
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "◀", // Add symbol
        textSize: 12,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.inverseBtn_moveRight = function() {
  return {
    type: "container",
    size: {
      w: 1/5,
      h: 35
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "▶", // Add symbol
        textSize: 12,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.inverseBtn_moveUp = function() {
  return {
    type: "container",
    size: {
      w: 1/5,
      h: 35
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "▲", // Add symbol
        textSize: 12,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.inverseBtn_tiltL = function(xyz) {
  return {
    type: "container",
    size: {
      w: 1/5,
      h: 35
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {

        if(xyz == x){

        }
        if(xyz == y){

        }
        if(xyz == z){

        }
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "◣", // Add symbol
        textSize: 12,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.inverseBtn_tiltR = function(xyz) {
  return {
    type: "container",
    size: {
      w: 1/5,
      h: 35
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {

        if(xyz == x){

        }
        if(xyz == y){

        }
        if(xyz == z){

        }
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "◥", // Add symbol
        textSize: 12,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.inverseBtn_zDown = function() {
  return {
    type: "container",
    size: {
      w: 1/5,
      h: 35
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "▼", // Add symbol
        textSize: 12,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.inverseBtn_zUp = function() {
  return {
    type: "container",
    size: {
      w: 1/5,
      h: 35
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "▲", // Add symbol
        textSize: 12,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.inverseColL = function() {
  var c = tr.controls.pnp2;

  var children = [];
  children.push(c.inverseBtn_Blnk());     children.push(c.inverseBtn_moveUp());   children.push(c.inverseBtn_Blnk()); children.push(c.inverseBtn_Blnk()); children.push(c.inverseBtn_zUp());
  children.push(c.inverseBtn_moveLeft()); children.push(c.inverseBtn_Blnk());     children.push(c.inverseBtn_moveRight()); children.push(c.inverseBtn_Blnk()); children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_Blnk());     children.push(c.inverseBtn_moveDown()); children.push(c.inverseBtn_Blnk()); children.push(c.inverseBtn_Blnk()); children.push(c.inverseBtn_zDown());
  return {
    type: "container",
    size: {
      w: 1/3,
      h: 0.3
      ,
    },
    background: "rgba(255, 255, 255, 0.2)",
    margin: 2,
    padding: 5,
    children: [{
      type: "container",
      border: false,
      children: children,
}]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.inverseColM = function() {
  var c = tr.controls.pnp2;

  var children = [];
  children.push(c.inverseBtn_tiltR("x"));children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_tiltR("y"));children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_tiltR("z"));
  children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_tiltL("x"));children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_tiltL("y"));children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_tiltL("z"));
  return {
    type: "container",
    size: {
      w: 1/3,
      h: 0.3
      ,
    },
    background: "rgba(255, 255, 255, 0.2)",
    margin: 2,
    padding: 5,
    children: [{
      type: "container",
      border: false,
      children: children,
}]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.inverseColR = function() {
  var c = tr.controls.pnp2;

  var children = [];
  children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_Blnk());
  children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_Blnk());children.push(c.inverseBtn_Blnk());
  return {
    type: "container",
    size: {
      w: 1/3,
      h: 0.3
      ,
    },
    background: "rgba(255, 255, 255, 0.2)",
    margin: 2,
    padding: 5,
    children: [{
      type: "container",
      border: false,
      children: children,
}]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.joystick = function() {
  return {
    type: "container",
    size: {
      w: 100,
      h: 100
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "joystick",
        size: {
          w: 100,
          h: 100
        }
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.programBtn_Add = function() {
  return {
    type: "container",
    size: {
      w: 1 / 4,
      h: 50
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        //Do program Add
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "+", // Add symbol
        textSize: 24,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.programBtn_Delete = function() {
  return {
    type: "container",
    size: {
      w: 1 / 4,
      h: 50
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        //Do program Delete
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "♲", //recycle symbol
        textSize: 24,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.programBtn_Pause = function() {
  return {
    type: "container",
    size: {
      w: 1 / 3,
      h: 40
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        //Do program play
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "II", // Right symbol
        textSize: 24,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.programBtn_Play = function() {
  return {
    type: "container",
    size: {
      w: 1 / 3,
      h: 40
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        //Do program play
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "▶", // Right symbol
        textSize: 24,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.programBtn_Settings = function() {
  return {
    type: "container",
    size: {
      w: 1 / 4,
      h: 50
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        //Do program Settings
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "S", // ⚙ Gear symbol Not Working
        textSize: 24,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.programBtn_Stop = function() {
  return {
    type: "container",
    size: {
      w: 1 / 3,
      h: 40
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        //Do program play
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "■", // Right symbol
        textSize: 24,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.programHeader = function() {
  var c = tr.controls.pnp2;

  var children = [];

  children.push(c.programSelect());
  children.push(c.programBtn_Delete());
  children.push(c.programBtn_Add());
  children.push(c.programBtn_Settings());

  children.push(c.waypointBtn_Previous());
  children.push(c.waypointDisplay());
  children.push(c.waypointBtn_Next());
  children.push(c.waypointBtn_Delete());
  children.push(c.waypointBtn_Add());
  return {
    type: "container",
    border:false,
    size:{
      w: 1,
      h: 100,
    },
    children: children,
    }
  }
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.programSelect = function() {
  return {
    type: "container",
    size: {
      w: 1 / 4,
      h: 50
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        //Do program Settings
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        id: "progselect",
        type: "select",
        size: {
          w: 1,
          h: 40
        },
        padding: 5,
        options: ["Did Not Load", ],
        textSize: 22,
        onChange: function(val) {
          var app = this.getApp();
        //  app.config.changeProgram(val);
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.render = function() {
  return {
    type: "container",
    border:false,
    size:{
      w: 1,
      h: 0.9,
    },
    children: [{
    type: "tr2",
      }],
    }
  }
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.pnp2.slider = function(id) {
  return {
    type: "container",
    size: {
      w: 5 / 9,
      h: 20
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: id + "slider",
        type: "slider",

        onDraw: function() {
          var app = this.getApp();
          var page = app.getCurrentPage();
          var sliderVal = page.getChild(id + "slider").element.value();
        },

        onInput: function(val) {
          var app = this.getApp();
          var page = app.getCurrentPage();
          var sliderVal = page.getChild(id + "slider").element.value();

          var val = 0;
          val = sliderVal * Math.PI * 2.0;
          //tr.data.socket.emit("/tr3/joints/" + id + "/control/position", val);
          var label = page.getChild(id + "sliderl");
          label.text = val.toFixed(2);
        },

        onChange: function(val) {
          var app = this.getApp();
          var page = app.getCurrentPage();
        },

        min: -1,
        max: 1,
        val: 0,
        step: 0.01,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.pnp2.sliderID_Label = function(id) {
  return {
    type: "container",
    size: {
      w: 2 / 9,
      h: 20
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: id,
        textSize: 14,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.sliderRow = function(id) {
  var c = tr.controls.pnp2;

  var children = [];

  children.push(c.sliderID_Label(id));
  children.push(c.sliderValue_Label(id));
  children.push(c.slider(id));

  return {
    type: "container",
    border: false,
    size: {
      w: 1,
      h: 20,
    },
    background: "rgba(255, 255, 255, 0.2)",
    margin: 10,
    padding: 5,
    children: [{
      type: "container",
      border: false,
      children: children,
}]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.pnp2.sliderValue_Label = function(id) {
  return {
    type: "container",
    size: {
      w: 2 / 9,
      h: 20
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: id + "sliderl",
        type: "text",
        text: "0.00",
        textSize: 14,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.tabForward = function() {
  var c = tr.controls.pnp2;

  var children = [];

    children.push(c.sliderRow("a0"));
    children.push(c.sliderRow("a1"));
    children.push(c.sliderRow("a2"));
    children.push(c.sliderRow("a3"));
    children.push(c.sliderRow("a4"));

  return {
    type: "container",
    border:false,
    size:{
      w: 1,
      h: 120,
    },
    children: children,
    }
  }
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.tabInverse = function() {
  var c = tr.controls.pnp2;

  var children = [];
  children.push(c.inverseColL())
  children.push(c.inverseColM())
  children.push(c.inverseColR())

  return {
    type: "container",
    border:false,
    size:{
      w: 1,
      h: 1,
    },
    children: children,
    }
  }
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.tabControl = function() {
  var c = tr.controls.pnp2;

  return {
    type: "container",
    border:false,
    size:{
      w: 1,
      h: 1,
    },
    children: [{
      type: "tabControl",
      labels: ["Forward", "Inverse"],
      pages: [c.tabForward(), c.tabInverse()],
    }]
    }
  }
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.waypointBtn_Add = function() {
  return {
    type: "container",
    size: {
      w: 1 / 5,
      h: 50
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        //Do waypoint Add
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "+", // Add symbol
        textSize: 24,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.waypointBtn_Delete = function() {
  return {
    type: "container",
    size: {
      w: 1 / 5,
      h: 50
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        //Do waypoint Delete
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "♲", //recycle symbol
        textSize: 24,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.waypointBtn_Next = function() {
  return {
    type: "container",
    size: {
      w: 1 / 5,
      h: 50
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        //Do waypoint previous
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "▶", // Right symbol
        textSize: 24,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.waypointBtn_Previous = function() {
  return {
    type: "container",
    size: {
      w: 1 / 5,
      h: 50
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        //Do waypoint previous
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "◀", // Left symbol
        textSize: 24,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.waypointDisplay = function() {
  return {
    type: "container",
    size: {
      w: 1 / 5,
      h: 50
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: "dwaypoint",
        type: "text",
        text: "0", // Current Waypoint ID
        textSize: 24,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
if (!tr) tr = {};
if (!tr.app) tr.app = {};

tr.app.pnp = new App({
  id: 2,
  name: "Pick N Place",
  desc: "Pick and Place",
  iconUrl: "/img/icon-app-pnp",

  programs: [],
  currentProgram: -1,
  waypointStart: 0,
  programMode: 0, // 0 = edit, 1, playback
  robotState: [],

  setup: function() {
    var app = this._app;
    app.addPages(tr.controls.pnp.pages);

    var page = app.pages[app.pageCurrent];
    page.onDraw = function() {
      this.app.config.programRun();
    };

    this.currentProgram = 0;
    this.programs.push(new tr.app.pnp.program({
      id: 0,
      name: "Program 0",
      waypoints: [{
        positions: [0, 0, 10, 0, 0],
        speed: 1
      }, {
        positions: [20, 20, 20, 20, 20],
        speed: 3
      }, {
        positions: [70, 70, 70, 70, 70],
        speed: 5
      }]
    }));

    this.programs.push(new tr.app.pnp.program({
      id: 1,
      name: "Program 1",
      waypoints: [{
        positions: [0, 0, 20, 0, 0],
        speed: 3
      }, {
        positions: [80, 80, 80, 80, 80],
        speed: 6
      }, {
        positions: [90, 40, 30, 60, 10],
        speed: 2
      }]
    }));

    this.robotState = this.getCurrentProgram().getCurrentWaypoint().positions;
    this.updateUI();
  },

  addProgram: function() {
    var id = 0;
    for (var i = 0; i < this.programs.length; i++) {
      if (this.programs[i].id >= id) {
        id = this.programs[i].id + 1;
      }
    }

    this.programs.push(new tr.app.pnp.program({
      id: id
    }));

    this.updateUI();
  },

  changeProgram: function(name) {
    for (var i = 0; i < this.programs.length; i++) {
      if (this.programs[i].name == name) {
        this.currentProgram = i;
        this.updateUI();
      }
    }
  },

  getCurrentProgram: function() {
    return this.programs[this.currentProgram];
  },

  programStart: function() {
    var prog = this.getCurrentProgram();
    prog.currentWaypoint = 0;
    var wp = prog.getCurrentWaypoint().positions;
    this.waypointStartPos = Object.assign([], wp);
    if (prog.waypoints.length > 2) {
      prog.currentWaypoint += 1;
      this.programMode = 1;
    }
  },

  programStartFrom: function() {
    var prog = this.getCurrentProgram();
    var wp = prog.getCurrentWaypoint().positions;
    this.waypointStartPos = Object.assign([], wp);
    if (prog.currentWaypoint < prog.waypoints.length - 1) {
      prog.currentWaypoint += 1;
      this.programMode = 1
    } else {
      this.programMode = 0;
    }
  },

  programStop: function() {
    this.programMode = 0;
  },

  programRun: function() {
    if (this.programMode == 1) {
      var prog = this.getCurrentProgram();
      var wp = prog.getCurrentWaypoint();
      var pos = wp.positions;
      var wpDuration = wp.speed; // seconds

      var startPos = this.waypointStartPos;

      var time = new Date();
      var duration = (time - this.waypointStart) / 1000.0;
      var durationComplete = (duration / wpDuration)

      if (wpDuration == 0) {
        durationComplete = 1.0;
      }

      for (var i = 0; i < this.robotState.length; i++) {
        this.robotState[i] = (pos[i] - startPos[i]) * durationComplete + startPos[i];
      }

      this.updateUI();

      if (duration >= wpDuration) {
        if (prog.waypoints.length - 1 <= prog.currentWaypoint) {
          this.programMode = 0;
        } else {
          this.waypointStart = new Date();
          this.waypointStartPos = Object.assign([], pos);
          prog.currentWaypoint += 1;
        }
      }
    } else {
      var pos = this.getCurrentProgram().getCurrentWaypoint().positions;
      this.waypointStart = new Date();
      this.waypointStartPos = Object.assign([], pos);
      this.robotState = Object.assign([], pos);
      this.updateUI();
    }
  },

  updateUI: function() {
    var prog = this.getCurrentProgram();
    var positions = prog.getCurrentWaypoint().positions;
    var speed = prog.getCurrentWaypoint().speed;

    var page = this._app.getCurrentPage();
    var tr2 = page.getChild('tr').tr2;
    tr2.state.a0 = this.robotState[0];
    tr2.state.a1 = this.robotState[1];
    tr2.state.a2 = this.robotState[2];
    tr2.state.a3 = this.robotState[3];
    tr2.state.a4 = this.robotState[4];

    page.getChild('dwaypoint').text = prog.currentWaypoint;
    // page.getChild('dspeed').text = Math.round(speed * 100) / 100;
    // page.getChild('d0').text = Math.floor(positions[0]) + "°";
    // page.getChild('d1').text = Math.floor(positions[1]) + "°";
    // page.getChild('d2').text = Math.floor(positions[2]) + "°";
    // page.getChild('d3').text = Math.floor(positions[3]) + "°";
    // page.getChild('d4').text = Math.floor(positions[4]) + "°";

    this.updateSelect();
  },

  updateSelect: function() {
    var page = this._app.pages[this._app.pageCurrent];
    var sel = page.getChild('progselect');
    var options = sel.options;

    var opts = [];
    var update = false;
    for (i = 0; i < this.programs.length; i++) {
      opts.push(this.programs[i].name);

      if (options.length - 1 < i) {
        update = true;
      } else if (opts[i] != options[i]) {
        update = true;
      }
    }

    if (update) {
      page.getChild('progselect').setOptions(opts);
    }
  }
});
if (!tr) tr = {}
if (!tr.controls) tr.controls = {}
if (!tr.controls.pnp) tr.controls.pnp = {};
//boop//
tr.controls.pnp.pages = [{
  id: "PNP01",

  pos: {
    x: 0,
    y: 0
  },
  size: {
    w: 1.0,
    h: 1.0
  },
  header: {
    text: "P.N.P.",
  },
  children: [{
    id: "colLeft",
    type: "container",
    align: {
      v: "CENTER",
      h: "CENTER"
    },
    size: {
      w: 0.5,
      h: 1
    },
    background: "rgba(255, 255, 255, 0.2)",
    margin: 10,
    padding: 5,
    children: [{
      id: "ProgramBar",
      type: "container",
      size: {
        w: 1.0,
        h: 1 / 8
      },
      background: "rgba(255, 255, 255, 0.5)",
      children: [{
        type: "container",
        size: {
          w: 1 / 4,
          h: "fill"
        },
        background: "rgba(255, 255, 255, 0.5)",
        children: [{
          id: "progselect",
          type: "select",
          size: {
            w: 1,
            h: 40
          },
          padding: 5,
          options: ["Did Not Load", ],
          textSize: 22,
          onChange: function(val) {
            var app = this.getApp();
            app.config.changeProgram(val);
          },
        }],

      }, {
        type: "container",
        size: {
          w: 1 / 4,
          h: "fill"
        },
        background: "rgba(255, 255, 255, 0.5)",
        children: [{
          align: {
            v: "CENTER",
            h: "CENTER"
          },
          type: "image",
          url: "/img/pnp-control-del0.png",
        }],
      }, {
        type: "container",
        size: {
          w: 1 / 4,
          h: "fill"
        },
        background: "rgba(255, 255, 255, 0.5)",
        children: [{
          align: {
            v: "CENTER",
            h: "CENTER"
          },
          type: "image",
          url: "/img/pnp-control-new0.png",
          onClick: function() {
            console.log("New Program");
            var app = this.getApp();
            app.config.addProgram();
          },
        }],
      }, {
        type: "container",
        size: {
          w: 1 / 4,
          h: "fill"
        },
        background: "rgba(255, 255, 255, 0.5)",
        children: [{
          align: {
            v: "CENTER",
            h: "CENTER"
          },
          type: "image",
          url: "/img/pnp-control-set0.png",
        }],
      }],



    }, {
      type: "container",
      size: {
        w: 1,
        h: 1 / 10
      },
      background: "rgba(255, 255, 255, 0.5)",
      children: [{
        type: "container",
        size: {
          w: 1 / 5,
          h: "fill"
        },
        background: "rgba(255, 255, 255, 0.5)",
        children: [{
          align: {
            v: "CENTER",
            h: "CENTER"
          },
          type: "image",
          url: "/img/pnp-control-bw0.png",
          onClick: function() {
            var app = this.getApp();
            app.config.getCurrentProgram().incrementWaypoint(-1);
          },
        }],
      }, {
        type: "container",
        size: {
          w: 1 / 5,
          h: "fill"
        },
        background: "rgba(255, 255, 255, 0.5)",
        children: [{
          id: "dwaypoint",
          type: "text",
          align: {
            v: "CENTER",
            h: "CENTER"
          },
          text: "",
        }],
      }, {
        type: "container",
        size: {
          w: 1 / 5,
          h: "fill"
        },
        background: "rgba(255, 255, 255, 0.5)",
        children: [{
          align: {
            v: "CENTER",
            h: "CENTER"
          },
          type: "image",
          url: "/img/pnp-control-fw0.png",
          onClick: function() {
            var app = this.getApp();
            app.config.getCurrentProgram().incrementWaypoint(1);
          },
        }],
      }, {
        type: "container",
        size: {
          w: 1 / 5,
          h: "fill"
        },
        background: "rgba(255, 255, 255, 0.5)",
        children: [{
          align: {
            v: "CENTER",
            h: "CENTER"
          },
          type: "image",
          url: "/img/pnp-control-del1.png",
          onClick: function() {
            var app = this.getApp();
            var prog = app.config.getCurrentProgram();
            prog.removeWaypoint();
          },
        }],
      }, {
        type: "container",
        size: {
          w: 1 / 5,
          h: "fill"
        },
        background: "rgba(255, 255, 255, 0.5)",
        children: [{
          align: {
            v: "CENTER",
            h: "CENTER"
          },
          type: "image",
          url: "/img/pnp-control-add0.png",
          onClick: function() {
            var app = this.getApp();
            var prog = app.config.getCurrentProgram();
            prog.insertWaypoint();
          },
        }],


      }],
    }, {
      id: "DisplayBar",
      type: "container",
      size: {
        w: 1,
        h: 1 / 3
      },
      background: "rgba(255, 255, 255, 0.5)",
      children: [{
        type: "container",
        size: {
          w: 1 / 2,
          h: "fill"
        },
        background: "rgba(255, 255, 255, 0.5)",
        children: [{
          type: "container",
          border: true,
          size: {
            w: 1,
            h: 1 / 5
          },
          children: [{
            type: "text",
            text: "Goal Position"
          }],
        }, {
          type: "container",
          border: true,
          size: {
            w: 1,
            h: 1 / 5
          },
          children: [{
            type: "container",
            border: true,
            size: {
              w: 1 / 5,
              h: 1
            },
            children: [{
              type: "text",
              text: "X"
            }],
          }],
        }, {
          type: "container",
          border: true,
          size: {
            w: 1,
            h: 1 / 5
          },
          children: [{
            type: "container",
            border: true,
            size: {
              w: 1 / 5,
              h: 1
            },
            children: [{
              type: "text",
              text: "Y"
            }],
          }],

        }, {
          type: "container",
          border: true,
          size: {
            w: 1,
            h: 1 / 5
          },
          children: [{
            type: "container",
            border: true,
            size: {
              w: 1 / 5,
              h: 1
            },
            children: [{
              type: "text",
              text: "Z"
            }],
          }],

        }, {
          type: "container",
          border: true,
          size: {
            w: 1,
            h: 1 / 5
          },






        }],
      }, {
        type: "container",
        size: {
          w: 1 / 2,
          h: "fill"
        },
        background: "rgba(255, 255, 255, 0.5)",
        children: [],
      }],

    }, {
      // 1st //
      id: "ControlBar",
      type: "container",

      background: "rgba(255, 255, 255, 0.5)",
      size: {
        w: 1,
        h: 1 / 2.3
      },
      children: [{
        type: "container",
        background: "rgba(255, 255, 255, 0.5)",
        size: {
          w: 1 / 3,
          h: 1
        },
        children: [{
          type: "container",
          border: false,
          size: {
            w: 1,
            h: 1 / 5
          },
          children: [],

        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {

          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "image",
            url: "/img/pnp-control3.png",
            size: {
              w: 26,
              h: 35
            },
          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "image",
            url: "/img/pnp-control3.png",
            size: {
              w: 26,
              h: 35
            },
          }],

        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "image",
            url: "/img/pnp-control2.png",
            size: {
              w: 26,
              h: 35
            },
          }],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "image",
            url: "/img/pnp-control0.png",
            size: {
              w: 26,
              h: 35
            },
          }],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "image",
            url: "/img/pnp-control1.png",
            size: {
              w: 26,
              h: 35
            },
          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "text",
            text: "Z",
          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "image",
            url: "/img/pnp-control4.png",
            size: {
              w: 26,
              h: 35
            },
          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "image",
            url: "/img/pnp-control4.png",
            size: {
              w: 26,
              h: 35
            },
          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1,
            h: 1 / 5
          },
          children: [],




        }],
      }, {



        // 2nd //
        type: "container",
        background: "rgba(255, 255, 255, 0.5)",
        size: {
          w: 1 / 3,
          h: 1
        },
        children: [{
          border: false,
          type: "container",
          size: {
            w: 1,
            h: 1 / 5
          },
          children: [], //
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },

          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "image",
            url: "/img/pnp-control5.png",
            size: {
              w: 26,
              h: 35
            },
          }],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },

          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "image",
            url: "/img/pnp-control5.png",
            size: {
              w: 26,
              h: 35
            },

          }],
        }, {
          type: "container",

          size: {
            w: 1 / 5,
            h: 1 / 5
          },

          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "image",
            url: "/img/pnp-control5.png",
            size: {
              w: 26,
              h: 35
            },

          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            id: "rotationdisplay0",
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "text",
            text: "X",

          }],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            id: "rotationdisplay1",
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "text",
            text: "Y",

          }],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            id: "rotationdisplay2",
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "text",
            text: "Z",

          }],
          onClick: function() {
            console.log(this.size.w + 'y: ' + this.size.h)
          },
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },

          children: [{

            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "image",
            url: "/img/pnp-control6.png",
            size: {
              w: 26,
              h: 35
            },
          }],
        }, {
          type: "container",

          size: {
            w: 1 / 5,
            h: 1 / 5
          },

          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "image",
            url: "/img/pnp-control6.png",
            size: {
              w: 26,
              h: 35
            },

          }],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },

          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "image",
            url: "/img/pnp-control6.png",
            size: {
              w: 26,
              h: 35
            },

          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1,
            h: 1 / 5
          },
          children: [],
        }],
      }, {
        // 3rd //
        type: "container",
        background: "rgba(255, 255, 255, 0.5)",
        size: {
          w: 1 / 3,
          h: 1
        },
        children: [{
          border: false,
          type: "container",
          size: {
            w: 1,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "image",
            url: "/img/pnp-control3.png",
            size: {
              w: 26,
              h: 35
            },
          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "image",
            url: "/img/pnp-control3.png",
            size: {
              w: 26,
              h: 35
            },
          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            id: "speeddisplay",
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "text",
            text: "0",
          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            id: "incrimentdisplay",
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "text",
            text: "1",

          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "image",
            url: "/img/pnp-control4.png",
            size: {
              w: 26,
              h: 35
            },
          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "image",
            url: "/img/pnp-control4.png",
            size: {
              w: 26,
              h: 35
            },
          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1,
            h: 1 / 5
          },
          children: [],

        }],
      }]

    }],
  }, {
    id: "colRight",
    type: "container",
    size: {
      w: 0.5,
      h: 1
    },
    background: "rgba(255, 255, 255, 0.2)",
    margin: 10,
    padding: 5,
    children: [{
      type: "container",
      background: "rgba(255, 255, 255, 0.5)",
      size: {
        w: 1,
        h: 0.9
      },
      children: [{
        id: "tr",
        cameraPos: {
          x: 90,
          y: 0,
          z: 275
        },
        type: "tr2",
      }],
    }, {
      type: "container",
      background: "rgba(255, 255, 255, 0.5)",
      size: {
        w: 1,
        h: "fill"
      },
      children: [{
        type: "container",
        background: "rgba(255, 255, 255, 0.5)",
        size: {
          w: 1 / 3,
          h: 1
        },
        children: [{
          align: {
            v: "CENTER",
            h: "CENTER"
          },
          type: "image",
          url: "/img/pnp-control-play0.png",
          onClick: function() {
            var app = this.getApp();
            app.config.programStartFrom();
          },
        }],
      }, {
        type: "container",
        background: "rgba(255, 255, 255, 0.5)",
        size: {
          w: 1 / 3,
          h: 1
        },
        children: [{
          align: {
            v: "CENTER",
            h: "CENTER"
          },
          type: "image",
          url: "/img/pnp-control-pause0.png",
          onClick: function() {
            var app = this.getApp();
            app.config.programStop();
          },
        }],
      }, {
        type: "container",
        background: "rgba(255, 255, 255, 0.5)",
        size: {
          w: 1 / 3,
          h: 1
        },
        children: [{
          align: {
            v: "CENTER",
            h: "CENTER"
          },
          type: "image",
          url: "/img/pnp-control-stop0.png",
          onClick: function() {
            var app = this.getApp();
            app.config.programStop();
          },

        }],



      }],
    }],
  }], // Page01 Close

}]
if (!tr) tr = {};
if (!tr.app) tr.app = {};
if (!tr.app.pnp) tr.app.pnp = {};

tr.app.pnp.waypoint = function(config) {
  this.config = config;
  if (!this.config) this.config = {};
  this.positions = this.config.positions || [0, 0, 0, 0, 0];
  this.speed = this.config.speed || 1;

  this.config.positions = this.positions;
  this.config.speed = this.speed;

  this.incrementPosition = function(idx, i) {
    this.positions[idx] += i;
  }

  this.incrementDuration = function(i) {
    this.speed += i;
    if (this.speed < 0) {
      this.speed = 0;
    }
  }
}

tr.app.pnp.program = function(config) {
  this.config = config || {};
  this.id = config.id || 0;
  this.name = config.name || "Program " + this.id;
  this.waypoints = [];
  this.currentWaypoint = -1;

  this.setup = function() {
    this.config.waypoints = this.config.waypoints || [];
    for (var i = 0; i < this.config.waypoints.length; i++) {
      var wp = this.config.waypoints[i];
      if (wp.config) {
        this.waypoints.push(wp);
      } else {
        this.waypoints.push(new tr.app.pnp.waypoint(wp));
      }
    }

    if (this.waypoints.length > 0) {
      this.currentWaypoint = 0;
    }
  }

  this.getCurrentWaypoint = function() {
    return this.waypoints[this.currentWaypoint];
  }

  this.insertWaypoint = function() {
    var wp = [];
    console.log(this);
    for (var i = 0; i < this.waypoints.length; i++) {
      wp.push(this.waypoints[i]);
      if (i == this.currentWaypoint) {
        var config = Object.assign({}, this.waypoints[i].config);
        config.positions = Object.assign([], config.positions);
        wp.push(new tr.app.pnp.waypoint(config));
      }
    }

    if (wp.length == 0) {
      wp.push(new tr.app.pnp.waypoint());
      this.currentWaypoint = 0;
    } else {
      this.currentWaypoint += 1;
    }

    this.waypoints = wp;
  }

  this.removeWaypoint = function() {
    var wp = [];
    for (var i = 0; i < this.waypoints.length; i++) {
      if (i != this.currentWaypoint) {
        wp.push(this.waypoints[i]);
      }
    }
    this.waypoints = wp;

    if (this.currentWaypoint >= this.waypoints.length) {
      this.currentWaypoint = this.waypoints.length - 1;
    }

    if (this.waypoints.length == 0) {
      this.insertWaypoint();
    }
  }

  this.incrementWaypoint = function(i) {
    this.currentWaypoint += i;
    if (this.currentWaypoint < 0) {
      this.currentWaypoint = 0;
    } else if (this.currentWaypoint >= this.waypoints.length) {
      this.currentWaypoint = this.waypoints.length - 1;
    }
  }

  this.setup();
}
if (!tr) tr = {};
if (!tr.app) tr.app = {};

tr.app.settings = new App({
  id: 4,
  name: "Settings",
  iconUrl: "/img/icon-app-settings",
  pages: [{
    id: "main",
    pos: {
      x: 0,
      y: 0
    },
    size: {
      w: 1.0,
      h: 1.0
    },
    header: {
      text: "Settings",
    },
    children: [{
      type: "container",
      size: {
        w: 0.25,
        h: 1.0
      },
      margin: 10,
      padding: 10,
      background: "rgba(255, 255, 255, 0.2)",
      radius: 15,
      children: [],
    }, {
      type: "container",
      size: {
        w: 0.75,
        h: 1.0
      },
      margin: 10,
      padding: 10,
      background: "rgba(255, 255, 255, 0.2)",
      radius: 15,
      children: [],
    }],
  }],
});
if (!tr) tr = {};
if (!tr.app) tr.app = {};

tr.app.store = new App({
  enabled: false,

  ////////////////////////////////////////
  // vvv CONFIG USED BY CONSTRUCTOR vvv //
  ////////////////////////////////////////

  name: "App Store",
  iconUrl: "/img/icon-app-store",

  draw: function() {
    background(240);
    fill(0);
    text(this.name, 100, 50);
  },

  mousePressed: function() {
    this._app.close();
  },

  ////////////////////////////////////////
  // vvv USER DEFINED / HELPER DATA vvv //
  ////////////////////////////////////////


});
if (!tr) tr = {};
if (!tr.data) tr.data = {};

tr.data.socket = '';
tr.data.robotState = {
  header: {},
  name: [],
  position: [],
  velocity: [],
  effort: []
};

tr.data.depth = [];

tr.data.lidar = {
  angle_increment: 0,
  ranges: [],
}

tr.data.setup = function() {
  tr.data.socket = io('http://localhost:8080/');

  tr.data.socket.on('/tr3/state', function(data) {
    tr.data.robotState = data;
  });

  tr.data.socket.on('/tr3/lidar', function(data) {
    tr.data.lidar = data;
  });

  tr.data.socket.on('/tr3/base/odom', function(data) {
    tr.data.odom = data;
  });

  tr.data.socket.on('/tr3/depth', function(data) {
    tr.data.depth = data;
  });

  tr.data.socket.on('/map', function(data) {
    tr.data.map = data;
  });
}

tr.data.getState = function(aid) {
  for (var i = 0; i < tr.data.robotState.name.length; i++) {
    if (tr.data.robotState.name[i] == aid) {
      return {
        position: tr.data.robotState.position[i],
        velocity: tr.data.robotState.velocity[i],
        effort: tr.data.robotState.effort[i]
      }
    }
  }

  return {};
}
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
    this.onMousePress = config.onMousePress;
    this.onMouseRelease = config.onMouseRelease;
    this.onDraw = config.onDraw;

    if (this.componentConfig.defaults) {
      this.componentConfig.defaults.bind(this)();
    }

    var parentWidth = (this.parent.size.w - this.parent.margin * 2.0 - this.parent.padding * 2.0);
    var parentHeight = (this.parent.size.h - this.parent.margin * 2.0 - this.parent.padding * 2.0)

    this.size = Object.assign({}, this.size);
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

    if (this.config.size && this.config.size.w == "fill") {
      var parentWidth = (this.parent.size.w - this.parent.margin * 2.0 - this.parent.padding * 2.0);
      this.size.w = parentWidth - this.parent.translateState.x;
    }

    if (this.config.size && this.config.size.h == "fill") {
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

  this.clear = function() {
    for (var i = 0; i < this.children.length; i++) {
      if (this.children[i].clear) {
        this.children[i].clear();
      }

      if (this.children[i].componentConfig.clear) {
        this.children[i].componentConfig.clear.bind(this.children[i])();
      }
    }
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

        if (this.onMousePress) {
          this.onMousePress();
          this._mousePressed = true;
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

    if (this.onMouseRelease && this._mousePressed) {
      this._mousePressed = false;
      this.onMouseRelease();
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
tr.gui.camera = {
  defaults: function() {
    this.border = false;
    this.cameraImageUrl = 'http://' + location.hostname + ':8081/stream?topic=/camera/rgb/image_raw&type=ros_compressed';
    this.image = '';
    this.loadingImage = false;
    this.element = '';
  },

  setup: function() {
    this.loadingImage = true;
    this.element = createElement("img","");
    this.element.attribute("src", this.cameraImageUrl);
    this.element.style("user-select", "none");
    this.element.hide();
  },

  draw: function() {
    this.size = this.parent.size;

    var pos = this.getAbsolutePosition();
    this.element.position(pos.x, pos.y);
    this.element.size(this.size.w, this.size.h);
    this.element.show();
  },

  clear: function () {
    this.element.remove();
  },

  drawPixels: function() {
    this.image = createImage(tr.data.cameraImage.width, tr.data.cameraImage.height);
    this.image.loadPixels();

    var pxls = new Uint8Array(tr.data.cameraImage.data);
    for (var i = 0; i < this.image.height; i++) {
      for (var j = 0; j < this.image.width; j++) {
        var idx = (i * this.image.width + j) * 3;
        var r = pxls[idx];
        var g = pxls[idx + 1];
        var b = pxls[idx + 2];
        this.image.set(j, i, [r, g, b, 255]);
      }
    }

    this.image.updatePixels();
    this.image.resize(this.parent.size.w, this.parent.size.h);
    image(this.image, this.pos.x, this.pos.y);
  },
};
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
    this.p5.ambientLight(125);
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
tr.gui.container = {}
tr.gui.datePicker = {
  defaults: function() {
    this.dMonth = "00";
    this.dDay = "00";
    this.dYear = "0000";

    this.changed = function() {
      if (!this.initialized) return;
      if (this.onChange) {
        var d = moment(this.dYear + "-" + this.dMonth + "-" + this.dDay);
        this.onChange(d);
      }
    }
  },

  setup: function() {
    var days = [];
    for (var i = 1; i <= 31; i++) {
      if (i < 10) {
        days.push("0" + String(i));
      } else {
        days.push(String(i));
      }
    }

    var years = [(moment().year())]
    years.push(years[0] + 1);

    var container = new tr.gui.component(tr.gui.container);
    container.setup({
      border: false,
      parent: this,
      size: {
        w: 1,
        h: 40
      },
      padding: 5,
      children: [{
        type: "select",
        size: {
          w: 0.25,
          h: 1
        },
        defaultValue: moment().format("MM"),
        options: ["01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11", "12"],
        textSize: 22,
        onChange: function(val) {
          this.parent.parent.dMonth = val;
          this.parent.parent.changed();
        },
      }, {
        type: "text",
        text: "/",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
        size: {
          w: 0.1,
          h: 1
        },
        textSize: 22,
      }, {
        type: "select",
        size: {
          w: 0.25,
          h: 1
        },
        defaultValue: moment().format("DD"),
        options: days,
        textSize: 22,
        onChange: function(val) {
          this.parent.parent.dDay = val;
          this.parent.parent.changed();
        }
      }, {
        type: "text",
        text: "/",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
        size: {
          w: 0.1,
          h: 1
        },
        textSize: 22,
      }, {
        type: "select",
        size: {
          w: 0.3,
          h: 1
        },
        defaultValue: moment().format("YYYY"),
        options: years,
        textSize: 22,
        onChange: function(val) {
          this.parent.parent.dYear = val;
          this.parent.parent.changed();
        }
      }]
    });
    this.children.push(container);
  }
}
tr.gui.drawLink = function(link, r_x, r_y, r_z, x, y, z, animate, _p5) {
  if (!_p5) _p5 = window;

  _p5.push();
  _p5.scale(0.2);

  _p5.noStroke();

  _p5.translate(x, y, z);
  _p5.rotateX(r_x);
  _p5.rotateY(r_y);
  _p5.rotateZ(r_z);

  if (animate) {
    animate();
  }

  _p5.directionalLight(255, 255, 255, 500, -1250, -1250);
  _p5.ambientLight(125);
  _p5.ambientMaterial(40);
  _p5.model(link);

  _p5.pop();
}
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
tr.gui.image = {
  defaults: function() {
    this.border = false;
    this.url = this.config.url;
    this.image = loadImage(this.url);
    this.rotation = this.config.rotation || 0;
  },

  draw: function() {
    angleMode(DEGREES);

    this.translate(this.size.w / 2, this.size.h / 2);
    rotate(this.rotation);
    this.translate(-this.size.w / 2, -this.size.h / 2);
    this.image.resize(this.size.w, this.size.h);
    image(this.image, this.pos.x, this.pos.y);
  },
}
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
tr.gui.joystick = {
  defaults: function() {
    this.joystick;
    this.border = false;
    this.exists = false;
    this.offset = this.config.offset || {
      x: -7,
      y: -157
    };
    if (!this.config.radius){
      this.radius = 95;
    }
    if (!this.config.size.w) {
      this.size.w = 1;
    }
    if (!this.config.size.h) {
      this.size.h = 1;
    }

  },
  setup: function () {
  },

  mouseDragged: function (){
    if(this.exists == true){
      console.log(this.joystick.getY(1)," | ",this.joystick.getX(1))
      console.log(mouseX,mouseY)
    this.joystick.activateJoystick(true);
  }
},
  mouseReleased: function() {
    if(this.exists == true){
      console.log("FALSE")
   this.joystick.activateJoystick(false);
 }
 },
  draw: function() {

  this.translate(this.padding, this.padding);
  this.translate(this.offset.x, this.offset.y);
    var pos = this.getAbsolutePosition();
    pos.x += 50
    pos.y += 150
    if (this.exists == false){
    this.joystick = tr.gui.joystick._joystick(pos.x,pos.y,this.radius);
    this.exists = true;
  }
  this.joystick.render();
  this.joystick.update();
     this.translate(-this.padding, -this.padding);
     this.translate(-this.offset.x, -this.offset.y);

  },

  _joystick: function(x,y,r){
    this.pos = createVector(x, y);
     this.stickPos =createVector(x,y + 50);
     this.r = r;
     this.controls = false;
     this.finger;
     this.value = createVector(0,0);
     this.render = function() {
        stroke(255,255,255,50);
       strokeWeight(this.r / 20);
       fill(255,255,255,20);
       ellipse(this.pos.x,this.pos.y, this.r);
          stroke(255,255,255);
       strokeWeight(this.r/5);
       line(this.pos.x, this.pos.y,this.stickPos.x, this.stickPos.y);
     }
     this.position = function(x,y){
       this.pos = createVector(x, y);
        this.stickPos =createVector(x,y);
     }
     this.activateJoystick = function(activate) {
        this.finger = createVector(mouseX, mouseY);
        var distance = dist(this.finger.x,this.finger.y, this.pos.x,this.pos.y);
       if (distance < this.r / 2 && activate)  {
        this.controls = true;
        } else{
          this.stickPos.x= this.pos.x;
          this.stickPos.y= this.pos.y;
          this.controls = false;
        }
      }

      this.update = function(){
       if (this.controls){
          this.finger = createVector(mouseX, mouseY);
          this.stickPos = p5.Vector.sub(this.finger, this.pos);
          this.stickPos.limit(this.r/2);
          this.value.x = this.stickPos.x;
          this.value.y = this.stickPos.y;
          this.stickPos = p5.Vector.add(this.pos,this.stickPos);
        }
      }
      this.getValue = function(v){
        this.value = this.value.mult(v);

      }
    this.getX = function(){
        this.getValue(1);
        return this.value.x;
    }
    this.getY = function(){
        this.getValue(1);
        return this.value.y;
    }
    return this;
  }

};
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
    this.componentConfig.drawMap.bind(this)();
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

    stroke("orange");
    fill("orange");

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

  drawMap: function () {
    if (!tr.data.map) return;
    if (!tr.data.odom) return;

    var p = tr.data.odom.position;

    translate(this.center.x, this.center.y);
    rotateZ(tr.data.odom.orientation.z);

    stroke("white");
    fill("white");

    for (var i = 0; i < tr.data.map.length; i++) {
      var d = tr.data.map[i];
      var x = (d.x - p.x) * this.scale;
      var y = (d.y - p.y) * this.scale;
      var dist = sqrt((x * x) + (y * y));
      if (dist < this.radius - 1) {
        circle(x, -y, 1);
      }
    }

    rotateZ(-tr.data.odom.orientation.z);
    translate(-this.center.x, -this.center.y);
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

    var bodySize = {
      w: this.size.w,
      h: this.size.h
    };

    if (this.config.header) {
      this.config.header.id = "header";
      this.config.header.parent = this;
      this.children.push(new tr.gui.header(this.config.header));
      bodySize.h -= this.children[0].size.h;
    }

    var container = new tr.gui.component(tr.gui.container);
    container.setup({
      id: "body",
      parent: this,
      size: {
        w: bodySize.w,
        h: bodySize.h
      }
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

  this.draw = function() {
    background(this.background);
    this.drawChildren();

    if (this.onDraw) {
      this.onDraw();
    }
  }

  this.hideElements = function(p) {
    if (!p) p = this;
    if (!p.children) p.children = [];
    for (var i = 0; i < p.children.length; i++) {
      if (p.children[i].p5) {
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
      } else {
        this.translate(-shift.x, this.children[i - 1].size.h);
        shift.x = 0;
        shift.y += this.children[i - 1].size.h;
        this.children[i].draw();
        this.translate(0, this.children[i].size.h);
        shift.y += this.children[i].size.h;
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
tr.gui.rect = function(x, y, wx, wy, r) {
  beginShape();

  var startAngle = TWO_PI * 0.00;
  var endAngle = TWO_PI * 0.25;

  var corners = [
    [x + wx, y + wy],
    [x, y + wy],
    [x, y],
    [x + wx, y]
  ];
  for (var i = 0; i < corners.length; i++) {
    var cx = corners[i][0];
    var cy = corners[i][1];

    cx -= r * Math.sign(cx - (wx + x) + 1);
    cy -= r * Math.sign(cy - (wy + y) + 1);

    var resolution = 15;
    for (var j = startAngle; j <= endAngle; j += (TWO_PI * 0.25) / resolution) {
      var vx = cx + r * Math.cos(j);
      var vy = cy + r * Math.sin(j);
      vertex(vx, vy);
    }

    startAngle += TWO_PI * 0.25;
    endAngle += TWO_PI * 0.25;
  }

  endShape(CLOSE);
}
tr.gui.select = {
  defaults: function() {
    this.border = false;
    this.options = this.config.options || [];
    this.defaultValue = this.config.defaultValue || "";
    this.onChange = this.config.onChange;
    this.element = "";
    this.changed = function() {
      var val = this.element.value();
      if (this.onChange) {
        this.onChange(val);
      }
    }
  },

  setup: function() {
    this.setOptions = function(o) {
      if (this.element) {
        this.element.hide();
      }

      this.element = createSelect('');
      for (var i = 0; i < o.length; i++) {
        this.element.option(o[i]);
      }

      this.element.hide();
      this.element.changed(this.changed.bind(this));

      if (this.defaultValue) {
        this.element.value(this.defaultValue);
      }

      this.options = o
      this.changed();
    }.bind(this);

    this.setOptions(this.options);
  },

  draw: function() {
    var pos = this.getAbsolutePosition();
    this.element.position(pos.x, pos.y);
    this.element.size(this.size.w - this.padding * 2, this.size.h - this.padding * 2);
    this.element.show();
  },

  clear: function() {
    this.element.remove();
  },

}
tr.gui.slider = {
  defaults: function() {
    this.border = false;
    this.min = this.config.min;
    this.max = this.config.max;
    this.val = this.config.val;
    this.step = this.config.step || 1;
    this.element = createSlider(this.min, this.max, this.val, this.step);
    this.onChange = this.config.onChange;
    this.onInput = this.config.onInput;

    this.inputed = function() {
      var val = this.element.value();
      if (this.onInput) {
        this.onInput(val);
      }
    };

    this.changed = function() {
      var val = this.element.value();
      if (this.onChange) {
        this.onChange(val);
      }
    };

    this.setval = function(val) {
      this.element.value(val);
      this.onInput(val);
      this.onChange(val);
    };
  },

  setup: function() {
    this.element.hide();
    this.element.input(this.inputed.bind(this));
    this.element.changed(this.changed.bind(this));
  },

  draw: function() {
    var pos = this.getAbsolutePosition();
    this.element.position(pos.x, pos.y);
    this.element.size(this.size.w - this.padding * 2 - 3, this.size.h - this.padding * 2);
    this.element.show();
  },

  clear: function() {
    this.element.remove();
  },
}
tr.gui.tabControl = {
  defaults: function() {
    this.labels = this.config.labels || [];
    this.pages = this.config.pages || [];
    this.config.currentPage = this.config.currentPage || 0;
  },

  setup: function() {
    var buttons = this.componentConfig.createButtons.bind(this)();
    var page = this.config.pages[this.config.currentPage];
    var container = new tr.gui.component(tr.gui.container);
    container.setup({
      type: "container",
      border: false,
      parent: this,
      margin: this.margin,
      padding: this.padding,
      size: {
        w: 1,
        h: 1
      },
      children: [buttons, page]
    });
    this.children.push(container);
  },

  draw: function () {
    for (var i = 0; i < this.children[0].children[0].children.length; i++) {
      var b = this.children[0].children[0].children[i];
      if (i == this.config.currentPage) {
        b.background = "rgb(50, 50, 50)";
      } else {
        b.background = "rgb(100, 100, 100)";
      }
    }
  },

  createButtons: function() {
    var c = [];
    var w = 1 / this.config.pages.length;
    for (var i = 0; i < this.config.pages.length; i++) {
      var b = this.componentConfig.createButton(this.config.labels[i], w);

      b.onClick = function() {
        var i = this.index;

        var tc = this.parent.parent.parent;
        tc.config.currentPage = i;

        var c = tc.config.pages[i];
        c.parent = tc.children[0];

        var t = new tr.gui.component(tr.gui.container);
        t.setup(c);

        tc.children[0].children[1].clear();
        tc.children[0].children[1] = t;
      }
      c.push(b);
    }
    return {
      type: "container",
      background: "rgb(80, 80, 80)",
      size: {
        w: 1.0,
        h: 30
      },
      children: c,
    };
  },

  createButton: function(label, w) {
    return {
      type: "container",
      size: {
        w: w,
        h: 30
      },
      children: [{
        type: "container",
        border: false,
        children: [{
          type: "text",
          text: label,
          textSize: 18,
          size: {
            w: 1,
            h: 1
          },
          padding: 4,
          align: {
            v: "TOP",
            h: "CENTER"
          },
        }],
      }]
    };
  }
}
tr.gui.text = {
  defaults: function() {
    this.border = false;
    this.softAlign = false;
    this.textFont = tr.fonts.roboto;

    if (!this.config.align) {
      this.config.align = {};
    }
    if (!this.config.align.v) {
      this.align.v = "TOP";
    }
    if (!this.config.align.h) {
      this.align.h = "LEFT";
    }

    if (!this.config.size) {
      this.config.size = {};
    }
    if (!this.config.size.w) {
      this.size.w = 1;
    }
    if (!this.config.size.h) {
      this.size.h = 1;
    }
  },

  setup: function () {
    if (this.config.textFont) {
      this.textFont = tr.fonts[this.config.textFont];
    }
  },

  draw: function() {
    stroke(this.textColor);
    fill(this.textColor);
    textFont(this.textFont);
    textSize(this.textSize);
    textAlign(window[this.align.h], window[this.align.v]);
    text(this.text, this.pos.x, this.pos.y, this.size.w - this.padding * 2, this.size.h - this.padding * 2);
  }
};
tr.gui.toolbar = {
  defaults: function() {
    this.border = false;
    this.size = {
      w: 1.0,
      h: 40
    };
    this.pos = this.config.pos || {
      x: 0,
      y: 0
    };
    this.align = {
      v: "TOP",
      h: "Left"
    };
    this.background = this.config.background || "rgba(255,255,0,0.5)";

    this.toolbarItems = this.config.children;
    this.config.children = [];
    this.children = [];
  },

  setup: function() {

    var toolbarcontainer = new tr.gui.component(tr.gui.container);
    var toolbarcontainersetup = {
      id: "Ctoolbar",
      border: false,
      parent: this,
      size: {
        w: 1.0,
        h: 40
      },
      align: {
        v: "TOP",
        h: "Left"
      },
      background: this.background,
      pos: this.pos,
      children: []
    };

    for (var i = 0; i < this.toolbarItems.length; i++) {
      var item = this.toolbarItems[i];
      toolbarcontainersetup.children.push({

        id: 'TB' + i,
        type: "container",
        margin: 0,
        align: {
          v: "TOP",
          h: "Left"
        },
        size: {
          w: 1 / this.toolbarItems.length,
          h: 40
        },
        background: "rgba(255,255,255,.8)",
        children: [{
          type: "image",
          size: {
            w: 1,
            h: 40
          },
          align: {
            v: "TOP",
            h: "Left"
          },
          url: item.img
        }],
        onClick: item.onClick
      })
    }
    toolbarcontainer.setup(toolbarcontainersetup);

    this.children.push(toolbarcontainer);
  },
}
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

    this.p5 = new p5(function(p) {}, this.container.id);
    this.p5.createCanvas(this.size.w, this.size.h, WEBGL);

    this.p5.angleMode(RADIANS);
    this.p5.perspective();

    this.links.b0 = this.p5.loadModel("/stl/tr-bs-a.stl");
    this.links.w0 = this.p5.loadModel("/stl/xt-wl-a.stl");
    this.links.a0 = this.p5.loadModel("/stl/xt-lg-b.stl");
    this.links.a1 = this.p5.loadModel("/stl/xt-lg-c.stl");
    this.links.a2 = this.p5.loadModel("/stl/xt-lg-b.stl");
    this.links.a3 = this.p5.loadModel("/stl/xt-sm-c.stl");
    this.links.a4 = this.p5.loadModel("/stl/xt-sm-b.stl");
    this.links.g0 = this.p5.loadModel("/stl/xt-gp-a.stl");
    this.links.g1 = this.p5.loadModel("/stl/xt-gp-b.stl");
    this.links.h0 = this.p5.loadModel("/stl/xt-hd-a.stl");
    this.links.h1 = this.p5.loadModel("/stl/xt-hd-b.stl");
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

      this.links.b0 = this.p5.loadModel("/stl/tr-bs-a.stl");
      this.links.w0 = this.p5.loadModel("/stl/xt-wl-a.stl");
      this.links.a0 = this.p5.loadModel("/stl/xt-lg-b.stl");
      this.links.a1 = this.p5.loadModel("/stl/xt-lg-c.stl");
      this.links.a2 = this.p5.loadModel("/stl/xt-lg-b.stl");
      this.links.a3 = this.p5.loadModel("/stl/xt-sm-c.stl");
      this.links.a4 = this.p5.loadModel("/stl/xt-sm-b.stl");
      this.links.g0 = this.p5.loadModel("/stl/xt-gp-a.stl");
      this.links.g1 = this.p5.loadModel("/stl/xt-gp-b.stl");
      this.links.h0 = this.p5.loadModel("/stl/xt-hd-a.stl");
      this.links.h1 = this.p5.loadModel("/stl/xt-hd-b.stl");
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

    this.p5.stroke("white");
    for (var i = 0; i < tr.data.depth.length; i++) {
      var d = tr.data.depth[i];
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
if (!tr) tr = {};
if (!tr.lib) tr.lib = {};

tr.lib.link = function(config) {
  this.id = config.id || "j0";
  this.meshId = config.meshId || this.id;
  this.offset = config.offset || 0;
  this.axis = config.axis || "Y";
  this.flip = config.flip || false;
  this.fixed = config.fixed || false;

  this.meshOffset = config.meshOffset || {
    x: 0,
    y: 0,
    z: 0
  }

  this.rotate = config.rotate || {
    x: 0,
    y: 0,
    z: 0
  }

  this.translate = config.translate || {
    x: 0,
    y: 0,
    z: 0
  }
}

tr.lib.tr2 = function() {
  this.head = [];
  this.arm = [];
  this.base = [];

  this.state = {};
  this.state.pos = {
    x: 0,
    y: 0,
    z: 0
  };
  this.state.a0 = 0;
  this.state.a1 = 0;
  this.state.a2 = 0;
  this.state.a3 = 0;
  this.state.a4 = 0;
  this.state.g0 = 0;
  this.state.h0 = 0;
  this.state.h1 = 0;

  this.setup = function() {
    // HEAD
    this.head.push(new tr.lib.link({
      id: "h0",
      meshId: "h0",
      axis: "Z",
      flip: true,
      rotate: {
        x: 0,
        y: 0,
        z: 1.5708
      },
      translate: {
        x: 0,
        y: 357.9,
        z: 703.25
      },
    }));

    this.head.push(new tr.lib.link({
      id: "h1",
      meshId: "h1",
      axis: "Z",
      flip: true,
      rotate: {
        x: -1.5708,
        y: 3.1415,
        z: -1.5708
      },
      translate: {
        x: -166.17,
        y: 67,
        z: 249.17
      },
    }));

    // ARM
    this.arm.push(new tr.lib.link({
      id: "a0",
      meshId: "a0",
      axis: "Z",
      rotate: {
        x: 0,
        y: 0,
        z: 0
      },
      translate: {
        x: 101.6,
        y: 122.9,
        z: 459.937
      },
      offset: 1.5708,
    }));

    this.arm.push(new tr.lib.link({
      id: "a1",
      meshId: "a1",
      axis: "Z",
      rotate: {
        x: -1.5708,
        y: 0,
        z: 0
      },
      translate: {
        x: 0,
        y: 70,
        z: 76
      },
      offset: 0,
    }));

    this.arm.push(new tr.lib.link({
      id: "a1_fixed",
      meshId: "a2",
      fixed: true,
      rotate: {
        x: 0,
        y: 0,
        z: 0
      },
      translate: {
        x: 0,
        y: 190,
        z: 0
      },
    }));

    this.arm.push(new tr.lib.link({
      id: "a2",
      meshId: "a3",
      axis: "Z",
      meshOffset: {
        x: 8,
        y: 300,
        z: 0
      },
      rotate: {
        x: 3.1415,
        y: 0,
        z: 1.5708
      },
      translate: {
        x: 0,
        y: 0,
        z: 0
      },
      offset: -0.698132,
    }));

    this.arm.push(new tr.lib.link({
      id: "a3",
      meshId: "a4",
      axis: "Z",
      flip: true,
      rotate: {
        x: 0,
        y: 3.1415,
        z: -1.5708
      },
      translate: {
        x: 8,
        y: 300,
        z: 0
      },
      offset: 0.698132,
    }));

    this.arm.push(new tr.lib.link({
      id: "a4",
      meshId: "g0",
      axis: "Y",
      flip: true,
      rotate: {
        x: 0,
        y: 0,
        z: 0
      },
      translate: {
        x: 0,
        y: 67+120,
        z: 67
      },
    }));
  }

  this.setup();
}
var canvasWidth = 864;
var canvasHeight = 480;

var appSelected = -1;

tr.fonts = {};

function preload() {
  tr.fonts.noto = loadFont('/ttf/noto.otf');
  tr.fonts.roboto = loadFont('/ttf/roboto.ttf');
}

function setup() {
  tr.data.setup();

  createCanvas(canvasWidth, canvasHeight, WEBGL);

  // for some stupid reason this is necessary to get certain font sizes to load correctly
  textFont(tr.fonts.roboto);
  textSize(2);
  text(".", 0, 0, 1, 1);

  desktop.setup(-1);

  desktop.apps = [];
  var apps = Object.getOwnPropertyNames(tr.app);
  for (var i = 0; i < apps.length; i++) {
    var a = tr.app[apps[i]];
    if (typeof a == "function") {
      a = a();
    }

    a.setup();

    if (a.enabled) {
      desktop.apps.push(a);
    }
  }

  desktop.apps = desktop.apps.sort(function(a, b) {
    return a.id - b.id;
  });
}

function draw() {
  translate(-canvasWidth / 2.0, -canvasHeight / 2.0);
  getSelectedApp().draw();
}

function mousePressed() {
  getSelectedApp().mousePressed();
}

function mouseReleased() {
  getSelectedApp().mouseReleased();
}

function mouseClicked() {
  getSelectedApp().mouseClicked();
}

function mouseDragged() {
  getSelectedApp().mouseDragged();
}

function keyPressed() {
  getSelectedApp().keyPressed();
}

function getSelectedApp() {
  if (appSelected < 0 || appSelected >= desktop.apps.length) {
    return desktop;
  } else {
    return desktop.apps[appSelected];
  }
}
