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
