var express = require('express');
var router = express.Router();
var passport = require('passport');
var LocalStrategy = require('passport-local').Strategy;
var fs = require('fs');
var fs2 = require('fs');
var fs3 = require('fs');
var fs4 = require('fs');
var fs5 = require('fs');
var User = require('../models/user');
var Settings = require('../models/settings');

// Register
router.get('/register', function(req, res){
	res.render('register');
});

// Login
router.get('/login', function(req, res){
	res.render('login');
});
// Settings
router.get('/settings', function(req, res){
	res.render('settings');
});

// Register User
router.post('/register', function(req, res){
	var name = req.body.name;
	var code = req.body.code;
	var username = req.body.username;
	var recalibrate = req.body.recalibrate;
	var password = req.body.password;
	var password2 = req.body.password2;

	// Validation
	req.checkBody('name', 'Name is required').notEmpty();
	req.checkBody('code', 'Code is required').notEmpty();
	req.checkBody('recalibrate', 'Recalibrate Setting is required').notEmpty();
	req.checkBody('username', 'Username is required').notEmpty();
	req.checkBody('password', 'Password is required').notEmpty();
	req.checkBody('password2', 'Passwords do not match').notEmpty();

	var errors = req.validationErrors();

	if(errors){
		res.render('register',{
			errors:errors
		});
	} else {
		var newUser = new User({
			name: name,
			code:code,
			username: username,
			password: password,
			recalibrate: recalibrate
		});
	User.createUser(newUser, function(err, user){
			if(err) throw err;
			console.log(user);
		});

		req.flash('success_msg', 'You are registered and can now login');

		res.redirect('/users/login');
	}
});

passport.use(new LocalStrategy(
  function(username, password, done) {
   User.getUserByUsername(username, function(err, user){
   	if(err) throw err;
   	if(!user){
   		return done(null, false, {message: 'Unknown User'});
   	}

   	User.comparePassword(password, user.password, function(err, isMatch){
   		if(err) throw err;
   		if(isMatch){
   			return done(null, user);
   		} else {
   			return done(null, false, {message: 'Invalid password'});
   		}
   	});
   });
  }));

passport.serializeUser(function(user, done) {
  done(null, user.id);
});

passport.deserializeUser(function(id, done) {
  User.getUserById(id, function(err, user) {
    done(err, user);
  });
});

router.post('/login',
  passport.authenticate('local', {successRedirect:'/', failureRedirect:'/users/login',failureFlash: true}),
  function(req, res) {
    res.redirect('/');
  });

router.get('/logout', function(req, res){
	req.logout();

	req.flash('success_msg', 'You are logged out');

	res.redirect('/users/login');
});

router.post('/settings', function(req, res){

	var setting1 = req.body.setting1;
	var setting2 = req.body.setting2;
	var setting3 = req.body.setting3;
	var setting4 = req.body.setting4;
	var setting5 = req.body.setting5;

	// Validation
	req.checkBody('setting1', 'setting 1 required').notEmpty();
	req.checkBody('setting2', 'setting 2 required').notEmpty();
	req.checkBody('setting3', 'setting 3 required').notEmpty();
	req.checkBody('setting4', 'setting 4 required').notEmpty();
	req.checkBody('setting5', 'setting 5 required').notEmpty();

	var errors = req.validationErrors();

	if(errors){
		res.render('settings',{
			errors:errors
		});
	} else {
		var newSettings = new Settings({
			setting1: setting1,
			setting2: setting2,
			setting3: setting3,
			setting4: setting4,
			setting5: setting5
		});
		fs.writeFile("/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/maxCount.txt",setting1, function(err) {
		 if(err) {
        console.log(err);
   		 } else {
        console.log("The file was saved!");
  		  }
		}); 
		fs2.writeFile("/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/enterpos.txt",setting2, function(err) {
		 if(err) {
        console.log(err);
   		 } else {
        console.log("The file was saved!");
  		  }
		}); 
		fs3.writeFile("/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/exitpos.txt",setting3, function(err) {
		 if(err) {
        console.log(err);
   		 } else {
        console.log("The file was saved!");
  		  }
		}); 
		fs4.writeFile("/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/maxcontour.txt",setting4, function(err) {
		 if(err) {
        console.log(err);
   		 } else {
        console.log("The file was saved!");
  		  }
		}); 
		fs5.writeFile("/home/e-motion/Software/DemoApplications/TinTin/pplcount/pal/loginapp-master/public/mincontour.txt",setting5, function(err) {
		 if(err) {
        console.log(err);
   		 } else {
        console.log("The file was saved!");
  		  }
		}); 


		

		req.flash('success_msg', 'You have changed your camera settings');

		res.redirect('/');
	}
});

module.exports = router;
