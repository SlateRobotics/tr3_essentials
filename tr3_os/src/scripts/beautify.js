var fs = require('fs');
var path = require('path');
var beautify = require('js-beautify').js;

var buildFileName = "tr.build.js";                                                                                               var root = path.join(__dirname, '../client/js/');

fs.readdir(root, function(err, files) {
  if (err) { return console.log(err) };

  files.forEach(function (file) {
    if (file.split('.')[0] == "tr" && file != buildFileName) {
      var data = fs.readFileSync(path.join(root, file), 'utf8');
      data = beautify(data, {indent_size: 2, end_with_newline: true});
      fs.writeFileSync(path.join(root, file), data);
    }
  });
});
