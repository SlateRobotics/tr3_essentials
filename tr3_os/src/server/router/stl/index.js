var express = require('express');
var router = express.Router();
var fs = require('fs');
var path = require('path');

var _root = '/../../../client/stl/';

router.get('*', function (req, res, next) {
	var fileName = path.join(__dirname, _root, '/' + req.originalUrl.replace('/stl/',''));
	res.sendFile(fileName);
});

module.exports = router;
