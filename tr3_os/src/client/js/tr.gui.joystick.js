tr.gui.joystick = {
  defaults: function() {
    this.joystick;
    this.border = false;
    this.exists = false;
    this.offset = this.config.offset || {
      x: -7,
      y: -157
    };
    if (!this.config.radius){
      this.radius = 95;
    }
    if (!this.config.size.w) {
      this.size.w = 1;
    }
    if (!this.config.size.h) {
      this.size.h = 1;
    }

  },
  setup: function () {
  },

  mouseDragged: function (){
    if(this.exists == true){
      console.log(this.joystick.getY(1)," | ",this.joystick.getX(1))
      console.log(mouseX,mouseY)
    this.joystick.activateJoystick(true);
  }
},
  mouseReleased: function() {
    if(this.exists == true){
      console.log("FALSE")
   this.joystick.activateJoystick(false);
 }
 },
  draw: function() {

  this.translate(this.padding, this.padding);
  this.translate(this.offset.x, this.offset.y);
    var pos = this.getAbsolutePosition();
    pos.x += 50
    pos.y += 150
    if (this.exists == false){
    this.joystick = tr.gui.joystick._joystick(pos.x,pos.y,this.radius);
    this.exists = true;
  }
  this.joystick.render();
  this.joystick.update();
     this.translate(-this.padding, -this.padding);
     this.translate(-this.offset.x, -this.offset.y);

  },

  _joystick: function(x,y,r){
    this.pos = createVector(x, y);
     this.stickPos =createVector(x,y + 50);
     this.r = r;
     this.controls = false;
     this.finger;
     this.value = createVector(0,0);
     this.render = function() {
        stroke(255,255,255,50);
       strokeWeight(this.r / 20);
       fill(255,255,255,20);
       ellipse(this.pos.x,this.pos.y, this.r);
          stroke(255,255,255);
       strokeWeight(this.r/5);
       line(this.pos.x, this.pos.y,this.stickPos.x, this.stickPos.y);
     }
     this.position = function(x,y){
       this.pos = createVector(x, y);
        this.stickPos =createVector(x,y);
     }
     this.activateJoystick = function(activate) {
        this.finger = createVector(mouseX, mouseY);
        var distance = dist(this.finger.x,this.finger.y, this.pos.x,this.pos.y);
       if (distance < this.r / 2 && activate)  {
        this.controls = true;
        } else{
          this.stickPos.x= this.pos.x;
          this.stickPos.y= this.pos.y;
          this.controls = false;
        }
      }

      this.update = function(){
       if (this.controls){
          this.finger = createVector(mouseX, mouseY);
          this.stickPos = p5.Vector.sub(this.finger, this.pos);
          this.stickPos.limit(this.r/2);
          this.value.x = this.stickPos.x;
          this.value.y = this.stickPos.y;
          this.stickPos = p5.Vector.add(this.pos,this.stickPos);
        }
      }
      this.getValue = function(v){
        this.value = this.value.mult(v);

      }
    this.getX = function(){
        this.getValue(1);
        return this.value.x;
    }
    this.getY = function(){
        this.getValue(1);
        return this.value.y;
    }
    return this;
  }

};
