var fs = require('fs')

/**
 * Randomize array element order in-place.
 * Using Durstenfeld shuffle algorithm.
 * From: https://stackoverflow.com/questions/2450954/how-to-randomize-shuffle-a-javascript-array
 */
function shuffleArray(array) {
    for (var i = array.length - 1; i > 0; i--) {
        var j = Math.floor(Math.random() * (i + 1));
        var temp = array[i];
        array[i] = array[j];
        array[j] = temp;
    }
    return array;
}


fs.readFile('all.json', 'utf8', function (err, data) {
    list = JSON.parse(data)
    list = shuffleArray(list)
    console.log(list)
    fs.writeFileSync('all.json',JSON.stringify(list))
  });