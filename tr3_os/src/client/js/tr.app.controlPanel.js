function label (id) {
  return {
    type: "container",
    size: {
      w: 0.1666,
      h: 25
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: id,
        textSize: 18,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}

function label_state (id) {
  return {
    type: "container",
    size: {
      w: 0.1666,
      h: 25
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: "",
        textSize: 18,
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

function button (id, label) {
  var i = 0;
  if (label == "--") {
    i = -1.0;
  } else if (label == "-") {
    i = -0.1;
  } else if (label == "+") {
    i = 0.1;
  } else if (label == "++") {
    i = 1.0;
  }

  return {
    type: "container",
    size: {
      w: 0.1666,
      h: 25
    },
    onClick: function() {
      tr.data.socket.emit("/tr3/joints/" + id + "/control/position", tr.data.getState(id).position + i);
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: label,
        textSize: 18,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}

function button_big (text, rostopic, value, background) {
  return {
    type: "container",
    size: {
      w: 0.5,
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

app.drawing = new App({
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
      type: "container",
      size: {
        w: 0.5,
        h: 1.0
      },
      margin: 10,
      padding: 10,
      background: "rgba(255, 255, 255, 0.2)",
      radius: 15,
      children: [
        label("a0"), label_state("a0"), button("a0", "--"), button("a0", "-"), button("a0", "+"), button("a0", "++"),
        label("a1"), label_state("a1"), button("a1", "--"), button("a1", "-"), button("a1", "+"), button("a1", "++"),
        label("a2"), label_state("a2"), button("a2", "--"), button("a2", "-"), button("a2", "+"), button("a2", "++"),
        label("a3"), label_state("a3"), button("a3", "--"), button("a3", "-"), button("a3", "+"), button("a3", "++"),
        label("a4"), label_state("a4"), button("a4", "--"), button("a4", "-"), button("a4", "+"), button("a4", "++"),
        label("h0"), label_state("h0"), button("h0", "--"), button("h0", "-"), button("h0", "+"), button("h0", "++"),
        label("h1"), label_state("h1"), button("h1", "--"), button("h1", "-"), button("h1", "+"), button("h1", "++"),
        button_big("STOP", "/tr3/stop", true, "rgba(212, 40, 40, 1)"), button_big("RELEASE", "/tr3/stop", false, "rgba(43, 212, 40, 1)"),
        button_big("SHUTDOWN", "/tr3/shutdown", true, "rgba(212, 40, 40, 1)"), button_big("POWER UP", "/tr3/powerup", true, "rgba(43, 212, 40, 1)")
      ],
    }, {
      type: "container",
      size: {
        w: 0.5,
        h: 1.0
      },
      margin: 10,
      padding: 10,
      background: "rgba(255, 255, 255, 0.2)",
      radius: 15,
      children: [{
        type: "tr2",
        onDraw: function() {
          //this.tr2.state.a0 += 1;
        }
      }],
    }],
  }],
});
