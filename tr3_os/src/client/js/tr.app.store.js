app.store = new App({

  ////////////////////////////////////////
  // vvv CONFIG USED BY CONSTRUCTOR vvv //
  ////////////////////////////////////////

  name: "App Store",
  iconUrl: "/img/icon-app-store",

  draw: function() {
    background(240);
    fill(0);
    text(this.name, 100, 50);
  },

  mousePressed: function() {
    this._app.close();
  },

  ////////////////////////////////////////
  // vvv USER DEFINED / HELPER DATA vvv //
  ////////////////////////////////////////


});
