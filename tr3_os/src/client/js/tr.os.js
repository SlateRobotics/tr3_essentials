var appSelected = -1;

tr.fonts = {};
tr.links = {};

function preload() {
  tr.fonts.noto = loadFont('/ttf/noto.otf');
  tr.fonts.roboto = loadFont('/ttf/roboto.ttf');

  tr.links.b0 = loadModel("/stl/tr-bs-a.stl");
  tr.links.w0 = loadModel("/stl/xt-wl-a.stl");
  tr.links.a0 = loadModel("/stl/xt-lg-b.stl");
  tr.links.a1 = loadModel("/stl/xt-lg-c.stl");
  tr.links.a2 = loadModel("/stl/xt-lg-b.stl");
  tr.links.a3 = loadModel("/stl/xt-sm-c.stl");
  tr.links.a4 = loadModel("/stl/xt-sm-b.stl");
  tr.links.g0 = loadModel("/stl/xt-gp-a.stl");
  tr.links.g1 = loadModel("/stl/xt-gp-b.stl");
  tr.links.h0 = loadModel("/stl/xt-hd-a.stl");
  tr.links.h1 = loadModel("/stl/xt-hd-b.stl");
}

var documentFullScreenInit = false;
document.onclick = function () {
  var fs = new URLSearchParams(window.location.search).get("fullscreen") || "1";
  if (!documentFullScreenInit && fs == "1") {
    document.documentElement.requestFullscreen();
    documentFullScreenInit = true;
  }
}

window.onload = function () {
  setCanvasSize();
  resizeCanvas(canvasWidth, canvasHeight);
}

var TR3_SCREEN_WIDTH = 1024;
var TR3_SCREEN_HEIGHT = 600;

var canvasWidth = TR3_SCREEN_WIDTH;
var canvasHeight = TR3_SCREEN_HEIGHT;

function setup() {
  tr.data.setup();

  setCanvasSize();
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

    if (a.enabled) {
      a.setup();
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

var lastPressEvent = '';
function mousePressed() {
  if (event.type == 'mousedown' && lastPressEvent == 'touchstart') return;
  getSelectedApp().mousePressed();
  lastPressEvent = event.type;
}

var lastReleaseEvent = '';
function mouseReleased() {
  if (event.type == 'mouseup' && lastReleaseEvent == 'touchend') return;
  getSelectedApp().mouseReleased();
  lastReleaseEvent = event.type;
}

function setCanvasSize () {
  var dev = new URLSearchParams(window.location.search).get("dev") || "0";
  if (dev == "0") {
    canvasWidth = windowWidth;
    canvasHeight = windowHeight;
  }

  if (canvasWidth < 768) canvasWidth = 768;
  if (canvasHeight < 450) canvasHeight = 450;
}

function windowResized() {
  setCanvasSize();
  resizeCanvas(canvasWidth, canvasHeight);
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
  if (appSelected < 0) {
    return desktop;
  } else {
    for (var i = 0; i < desktop.apps.length; i++) {
      if (desktop.apps[i].id == appSelected) {
        return desktop.apps[i];
      }
    }

    appSelected = -1;
    return desktop;
  }
}
