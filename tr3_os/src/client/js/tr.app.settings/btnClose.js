tr.controls.settings.btnClose = function() {
  return {
    id: "container-exit-fullscreen",
    type: "container",
    size: {
      w: 1,
      h: 125
    },
    margin: 15,
    background: "red",
    onClick: function() {
      if((window.fullscreen()) || (window.innerWidth == screen.width && window.innerHeight == screen.height)) {
        document.exitFullscreen();
        documentFullScreen = false;
      } else {
        document.documentElement.requestFullscreen()
        documentFullScreen = true;
      }
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: "btn-toggle-exit-fullscreen",
        type: "text",
        text: "EXIT FULLSCREEN",
        textSize: 42,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
        onDraw: function () {
          if((window.fullscreen()) || (window.innerWidth == screen.width && window.innerHeight == screen.height)) {
            this.text = "EXIT FULLSCREEN";
            this.parent.parent.background = "red";
          } else {
            this.text = "ENTER FULLSCREEN";
            this.parent.parent.background = "green";
          }
        },
      }],
    }]
  }
}
