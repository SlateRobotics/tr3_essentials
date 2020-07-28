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
