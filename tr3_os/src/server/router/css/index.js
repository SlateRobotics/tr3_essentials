var express = require('express');
var router = express.Router();
var fs = require('fs');
var path = require('path');

var _root = '/../../../client/css/';

router.get('*', function (req, res, next) {
	var fileName = path.join(__dirname, _root, '/' + req.originalUrl.replace('/css/',''));
	res.sendFile(fileName);
});

module.exports = router;
