tr.controls.controlPanel.gains = function(type) {
  var c = tr.controls.controlPanel;

  var children = [];

  children.push({
    type: "container",
    size: {
      w: 1.0,
      h: 60
    },
    background: "rgb(50,50,50)",
    children: [{
      type: "text",
      text: type.toUpperCase() + " GAINS",
      textSize: 18,
      align: {
        v: "CENTER",
        h: "CENTER"
      },
    }]
  })

  children.push(c.lblPID(type, "P"));
  children.push(c.btnPID(type, "P", "◀"))
  children.push(c.txtPID(type, "P"));
  children.push(c.btnPID(type, "P", "▶"))
  
  children.push(c.lblPID(type, "I"));
  children.push(c.btnPID(type, "I", "◀"))
  children.push(c.txtPID(type, "I"));
  children.push(c.btnPID(type, "I", "▶"))
  
  children.push(c.lblPID(type, "D"));
  children.push(c.btnPID(type, "D", "◀"))
  children.push(c.txtPID(type, "D"));
  children.push(c.btnPID(type, "D", "▶"))
  
  children.push(c.lblPID(type, "Dead Zone"));
  children.push(c.btnPID(type, "DZ", "◀"))
  children.push(c.txtPID(type, "DZ"));
  children.push(c.btnPID(type, "DZ", "▶"))
  
  children.push(c.lblPID(type, "I Clamp"));
  children.push(c.btnPID(type, "IC", "◀"))
  children.push(c.txtPID(type, "IC"));
  children.push(c.btnPID(type, "IC", "▶"))

  return {
    type: "container",
    size: {
      w: 0.333,
      h: "fill",
    },
    border: false,
    margin: 5,
    children: children,
  }
}
