var mongoose = require('mongoose');
var bcrypt = require('bcryptjs');

// User Schema
var SettingsSchema = mongoose.Schema({
	setting1: {
		type: String,
		index:true
	},
	setting3: {
		type: String
	},
	setting2: {
		type: String
	},
	setting4: {
		type: String
	},
	setting5: {
		type: String
	}
});

var Settings = module.exports = mongoose.model('Settings', SettingsSchema);

module.exports.createSettings = function(newSettings, callback){
	bcrypt.genSalt(10, function(err, salt) {
	    bcrypt.hash(newUser.password, salt, function(err, hash) {
	        newSettings.password = hash;
	        newSettings.save(callback);
	    });
	});
}

module.exports.getSettingsBySettings = function(setting1, callback){
	var query = {setting1: setting1};
	Settings.findOne(query, callback);
}

module.exports.getSettingsById = function(id, callback){
	Settings.findById(id, callback);
}


