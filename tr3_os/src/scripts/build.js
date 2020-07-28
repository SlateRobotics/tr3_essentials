var path = require('path');
var fs = require('fs');

var buildFileName = "tr.build.js";
var root = path.join(__dirname, '../client/js/');

fs.readdir(root, function(err, files) {
  if (err) { return console.log(err) };

  var build = "";
  files.forEach(function (file) {
    if (file.split('.')[0] == "tr" && file != buildFileName) {
      build += fs.readFileSync(path.join(root, file), 'utf8');
    }
  });

  fs.writeFileSync(path.join(root, buildFileName), build);
});
