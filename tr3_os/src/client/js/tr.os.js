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

var documentFullScreen = false;
document.onclick = function () {
  if (!documentFullScreen) {
    document.documentElement.requestFullscreen();
    documentFullScreen = true;
  }
}

function setup() {
  tr.data.setup();

  createCanvas(windowWidth, windowHeight, WEBGL);

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
  translate(-windowWidth / 2.0, -windowHeight / 2.0);
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

function windowResized() {
  resizeCanvas(windowWidth, windowHeight)
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
