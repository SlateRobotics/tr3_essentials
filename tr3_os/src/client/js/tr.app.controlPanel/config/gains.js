tr.controls.controlPanel.gains = function(type) {
  var c = tr.controls.controlPanel;

  var children = [];

  children.push({
    type: "container",
    size: {
      w: 1.0,
      h: 40
    },
    background: "rgb(50,50,50)",
    children: [{
      type: "text",
      text: type.toUpperCase(),
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
  
  children.push(c.lblPID(type, "IC"));
  children.push(c.btnPID(type, "IC", "◀"))
  children.push(c.txtPID(type, "IC"));
  children.push(c.btnPID(type, "IC", "▶"))
  
  children.push(c.lblPID(type, "FF"));
  children.push(c.btnPID(type, "FF", "◀"))
  children.push(c.txtPID(type, "FF"));
  children.push(c.btnPID(type, "FF", "▶"))
  
  children.push(c.lblPID(type, "MIN"));
  children.push(c.btnPID(type, "MIN", "◀"))
  children.push(c.txtPID(type, "MIN"));
  children.push(c.btnPID(type, "MIN", "▶"))
  
  children.push(c.lblPID(type, "MAX"));
  children.push(c.btnPID(type, "MAX", "◀"))
  children.push(c.txtPID(type, "MAX"));
  children.push(c.btnPID(type, "MAX", "▶"))

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
