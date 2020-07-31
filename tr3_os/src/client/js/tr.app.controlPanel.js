function label (id) {
  return {
    type: "container",
    size: {
      w: 0.111,
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
      w: 0.111,
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

function button_pos (id, label) {
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
      w: 0.111,
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

function button_mode (id, label) {
  var i = 0;
  if (label == "EFF") {
    i = 0;
  } else if (label == "BACK") {
    i = 1;
  } else if (label == "SRVO") {
    i = 2;
  }

  return {
    type: "container",
    size: {
      w: 0.111,
      h: 25
    },
    onClick: function() {
      tr.data.socket.emit("/tr3/joints/" + id + "/mode", i);
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
        w: 0.666,
        //w: 0.25,
        h: 1.0
      },
      margin: 10,
      padding: 10,
      background: "rgba(255, 255, 255, 0.2)",
      radius: 15,
      children: [
        label("ID"), label("POS"), label(""), label(""), label(""), label(""), label("MODE"), label("MODE"), label("MODE"),
        label("a0"), label_state("a0"), button_pos("a0", "--"), button_pos("a0", "-"), button_pos("a0", "+"), button_pos("a0", "++"), button_mode("a0", "EFF"), button_mode("a0", "BACK"), button_mode("a0", "SRVO"),
        label("a1"), label_state("a1"), button_pos("a1", "--"), button_pos("a1", "-"), button_pos("a1", "+"), button_pos("a1", "++"), button_mode("a1", "EFF"), button_mode("a1", "BACK"), button_mode("a1", "SRVO"),
        label("a2"), label_state("a2"), button_pos("a2", "--"), button_pos("a2", "-"), button_pos("a2", "+"), button_pos("a2", "++"), button_mode("a2", "EFF"), button_mode("a2", "BACK"), button_mode("a2", "SRVO"),
        label("a3"), label_state("a3"), button_pos("a3", "--"), button_pos("a3", "-"), button_pos("a3", "+"), button_pos("a3", "++"), button_mode("a3", "EFF"), button_mode("a3", "BACK"), button_mode("a3", "SRVO"),
        label("a4"), label_state("a4"), button_pos("a4", "--"), button_pos("a4", "-"), button_pos("a4", "+"), button_pos("a4", "++"), button_mode("a4", "EFF"), button_mode("a4", "BACK"), button_mode("a4", "SRVO"),
        label("h0"), label_state("h0"), button_pos("h0", "--"), button_pos("h0", "-"), button_pos("h0", "+"), button_pos("h0", "++"), button_mode("h0", "EFF"), button_mode("h0", "BACK"), button_mode("h0", "SRVO"),
        label("h1"), label_state("h1"), button_pos("h1", "--"), button_pos("h1", "-"), button_pos("h1", "+"), button_pos("h1", "++"), button_mode("h1", "EFF"), button_mode("h1", "BACK"), button_mode("h1", "SRVO"),
        { type: "container", size: { w: 1.0, h: 10 }, border: false },
        button_big("STOP", "/tr3/stop", true, "rgba(212, 40, 40, 1)"), button_big("RELEASE", "/tr3/stop", false, "rgba(43, 212, 40, 1)"),
        { type: "container", size: { w: 1.0, h: 10 }, border: false },
        button_big("SHUTDOWN", "/tr3/shutdown", true, "rgba(212, 40, 40, 1)"), button_big("POWER UP", "/tr3/powerup", true, "rgba(43, 212, 40, 1)")
      ],
    }, {
      type: "container",
      size: {
        w: 0.333,
        //w: 0.75,
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
