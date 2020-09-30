var express = require('express');
var router = express.Router();
var fs = require('fs');
var path = require('path');

var _root = '/../../../client/js/';

router.get('*', function (req, res, next) {
	var fileName = path.join(__dirname, _root, '/' + req.originalUrl.replace('/js/',''));

  if (fileName.includes("tr.build.js")) {
    require(path.join(__dirname, '../../../scripts/build.js'))();
  }

	res.sendFile(fileName);
});

module.exports = router;
