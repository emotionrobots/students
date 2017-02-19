// hello.js
const addon = require('./build/Release/addon');
while(true)
{
console.log(addon.hello());
}
// Prints: 'world'

