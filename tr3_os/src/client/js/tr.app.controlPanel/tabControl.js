if (!tr) tr = {};
if (!tr.app) tr.app = {};
if (!tr.app.controlPanel) tr.app.controlPanel = {};

tr.app.controlPanel.tabControl = function () {
  var c = tr.app.controlPanel;

  var children = [];
  children.push.apply(children, c.controlHeader());
  children.push.apply(children, c.controlRow("a0"));
  children.push.apply(children, c.controlRow("a1"));
  children.push.apply(children, c.controlRow("a2"));
  children.push.apply(children, c.controlRow("a3"));
  children.push.apply(children, c.controlRow("a4"));
  children.push.apply(children, c.controlRow("h0"));
  children.push.apply(children, c.controlRow("h1"));
  children.push({ type: "container", size: { w: 1.0, h: 10 }, border: false });
  children.push(c.btnBig("STOP", "/tr3/stop", true, "rgba(212, 40, 40, 1)"));
  children.push(c.btnBig("RELEASE", "/tr3/stop", false, "rgba(43, 212, 40, 1)"));
  children.push({ type: "container", size: { w: 1.0, h: 10 }, border: false });
  children.push(c.btnBig("SHUTDOWN", "/tr3/shutdown", true, "rgba(212, 40, 40, 1)"));
  children.push(c.btnBig("POWER UP", "/tr3/powerup", true, "rgba(43, 212, 40, 1)"));

  return {
    type: "container",
    size: {
      w: 1.0,
      h: "fill"
    },
    padding: 10,
    background: "rgba(255, 255, 255, 0.2)",
    children: children,
  }
}
