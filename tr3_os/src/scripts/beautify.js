var fs = require('fs');
var path = require('path');
var beautify = require('js-beautify').js;

var buildFileName = "tr.build.js";

function beautifyFiles (root, files) {
  files.forEach(function (file) {
    var stat = fs.statSync(path.join(root, file));
    if (stat && stat.isDirectory()) {
      var dir_files = fs.readdirSync(path.join(root, file));
      beautifyFiles(path.join(root, file), dir_files);
    } else {
      if (path.join(root, file).includes("tr.") && file != buildFileName) {
        var data = fs.readFileSync(path.join(root, file), 'utf8');
        data = beautify(data, {indent_size: 2, end_with_newline: true});
        fs.writeFileSync(path.join(root, file), data);
      }
    }
  });
}

var root = path.join(__dirname, '../client/js/');
var files = fs.readdirSync(root);
beautifyFiles(root, files);
