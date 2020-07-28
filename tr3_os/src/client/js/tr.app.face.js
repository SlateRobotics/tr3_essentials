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
