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
      document.exitFullscreen();
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
      }],
    }]
  }
}
