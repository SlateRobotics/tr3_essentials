var canvasWidth = 864;
var canvasHeight = 480;

var appSelected = -1;
var apps = [app.butler, app.pnp, app.chess, app.drawing, app.settings];

var font = "";

function setup() {
  tr.data.setup();
  font = loadFont('/ttf/roboto.ttf');
  textFont(font);

  createCanvas(canvasWidth, canvasHeight, WEBGL);

  desktop.setup(-1);
  desktop.apps = apps;

  apps = apps.sort(function(a, b) {
    return b.id - a.id;
  });

  for (var i = 0; i < apps.length; i++) {
    apps[i].setup(i);
  }
}

function draw() {
  textFont(font);
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
  if (appSelected < 0 || appSelected >= apps.length) {
    return desktop;
  } else {
    return apps[appSelected];
  }
}
