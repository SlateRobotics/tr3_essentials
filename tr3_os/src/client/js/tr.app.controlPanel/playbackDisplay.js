if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.playbackDisplay = function() {
  var c = tr.controls.controlPanel;

  var children = [];
  children.push(c.playbackLabel("a0"));
  children.push(c.playbackValue("a0"));
  children.push(c.playbackLabel("a4"));
  children.push(c.playbackValue("a4"));
  children.push(c.playbackLabel("a1"));
  children.push(c.playbackValue("a1"));
  children.push(c.playbackLabel("g0"));
  children.push(c.playbackValue("g0"));
  children.push(c.playbackLabel("a2"));
  children.push(c.playbackValue("a2"));
  children.push(c.playbackLabel("h0"));
  children.push(c.playbackValue("h0"));
  children.push(c.playbackLabel("a3"));
  children.push(c.playbackValue("a3"));
  children.push(c.playbackLabel("h1"));
  children.push(c.playbackValue("h1"));

  return {
    type: "container",
    size: {
      w: 3 / 9,
      h: 120,
    },
    padding: 10,
    background: "rgba(255, 255, 255, 0.2)",
    children: children,
  }
}
