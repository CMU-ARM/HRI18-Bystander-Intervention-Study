var express = require('express');
var app = express();
var path = require('path');
var fs = require('fs')
var bodyParser = require('body-parser')



// Define the port to run on
app.set('port', 3000);
//find the actual path
var learning_path = path.dirname(__filename)

var exphbs = require("express-handlebars"); 
app.engine('handlebars', exphbs({defaultLayout: 'main',layoutsDir:path.join(learning_path,'public/views/layouts')})); 
//app.engine('html', exphbs({defaultLayout: 'main'})); 

app.set('view engine', 'handlebars');
//app.set('view engine', 'html');


app.set('views', path.join(learning_path,'public','views'));
app.use(express.static(path.join(learning_path, 'public')));
app.use(bodyParser.urlencoded({ extended: false })) //use body parser
// Listen for requests

var server = app.listen(app.get('port'), function() {
  var port = server.address().port;
  console.log('Server listening on port ' + port);
});

//start io package
var io = require('socket.io')(server);

dt = new Date()
//log_name = process.argv[2]?process.argv[2]:'basic'
log_file_directory = process.argv[2]
var log_stream = fs.createWriteStream(path.join(log_file_directory,dt.getTime()+'.log'))

max_level = 8
app.get('/get_json',function(req,res){
  level = req.query.level ? req.query.level : 1
  player = req.query.player_id ? req.query.player_id : -1
  
  fs.readFile(path.join(learning_path, 'level_files','level_test_' + level + '.json'), 'utf8', function (err, data) {
    if (err){
      console.log("cannot find file")
      return fs.readFile(path.join('level_files','level_test_' + max_level + '.json'), 'utf8', function (err, data) {
        res.end(data)
      })
    }
    res.send(data)
    res.end()
  });
})

app.get('/admin', function(req, res){
  res.render('admin.handlebars',{'level':0})
})



app.post('/user_status',function(req, res){


  //log the required the information into write string 
  status = req.body
  dt = new Date()

  info_str = dt.getTime() + ',' + status["level"] + ',' + status["stage"] + ',' + status["index"] + ',' + status["score"] + ',' + status["last_guess"] + '\n'
  log_stream.write(info_str)

  //console.log(req.body)
  io.emit('status',req.body)
  res.end("")
})

app.get('/test', function(req, res)
{
  //console.log(req.query.level)
  level = req.query.level ? req.query.level : 1
  player = req.query.player_id ? req.query.player_id : 1
  res.render('test.handlebars',{'level':level,'player_id':player})
});

app.get('/learn', function(req, res){
  //get the current level
	level = req.query.level ? req.query.level : 1
  //read the level files
  fs.readFile(path.join(learning_path,'level_files','level_learn_' + level + '.json'), 'utf8', function (err, data) {
    level_file = {'text':[]}
    if(!err){
      level_file = JSON.parse(data)
      if(Object.keys(level_file).length === 0){
        //console.log("redirecting")
        return res.redirect('test?level=' + (parseInt(req.query.level)) + '&player_id=1')
      }
    }
    data = {
      level:level,
      words:level_file.text,
      learning_time:120//number of seconds
    }

	 res.render('learn.handlebars', data)
  })
})

app.all('*', function(req, res) {
  res.redirect("/learn?level=1&id=1");
});
