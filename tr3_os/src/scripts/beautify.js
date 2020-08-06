var fs = require('fs');
var path = require('path');
var beautify = require('js-beautify').js;

var buildFileName = "tr.build.js";
var root = path.join(__dirname, '../client/js/');

var files = fs.readdirSync(root);

files.forEach(function (file) {
  var stat = fs.statSync(path.join(root, file));
  if (stat && stat.isDirectory()) {
    var dir_files = fs.readdirSync(path.join(root, file));
    dir_files.forEach(function (dir_file) {
      if (file.split('.')[0] == "tr" && file != buildFileName) {
        var data = fs.readFileSync(path.join(root, file, dir_file), 'utf8');
        data = beautify(data, {indent_size: 2, end_with_newline: true});
        fs.writeFileSync(path.join(root, file, dir_file), data);
      }
    });
  } else {
    if (file.split('.')[0] == "tr" && file != buildFileName) {
      var data = fs.readFileSync(path.join(root, file), 'utf8');
      data = beautify(data, {indent_size: 2, end_with_newline: true});
      fs.writeFileSync(path.join(root, file), data);
    }
  }
});
