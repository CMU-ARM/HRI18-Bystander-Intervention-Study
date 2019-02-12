function padding(number, length){
  num_str = number.toString()
  while(num_str.length < length){
    num_str = '0' + num_str
  }
  return num_str
}

function updateTimer($, element_name, minutes, seconds){
  minutes = minutes < 0?0:minutes
  seconds = seconds < 0?0:seconds

  var text = padding(minutes,2) + ':' + padding(seconds,2)
  $(element_name).text(text)
}

function createTimer($, element_name, total_time, callback){

//get the start loading time
var start_time = new Date().getTime();

$(element_name).css('color','black')
$(element_name).css('font-weight','400')
var minutes = Math.floor((total_time % (1000 * 60 * 60)) / (1000 * 60));
var seconds = Math.floor((total_time % (1000 * 60)) / 1000);
updateTimer($,element_name, minutes, seconds)

//every second
var timer = setInterval(function(){
  var now = new Date().getTime()
  
  var passed_time = now - start_time
  //calculate remaining time
  var remaining_time = total_time - passed_time
  //check if we reach the end time
  if(passed_time > total_time){
    //clear the interval and move on
    clearInterval(timer)
    callback()
  }

  var minutes = Math.floor((remaining_time % (1000 * 60 * 60)) / (1000 * 60));
  var seconds = Math.floor((remaining_time % (1000 * 60)) / 1000);
  updateTimer($,element_name, minutes, seconds)
  if(passed_time > (total_time * 0.9)){
    $(element_name).css('color','red')
    $(element_name).css('font-weight','600')
  }
},500)
return timer
}


//var ros_url = 'ws://192.168.1.101:9090'
//var ros_url = 'ws://128.237.129.248:9090'
//var ros_url = 'ws://GS15619.SP.CS.CMU.EDU:9090'
//ros_url = 'ws://128.0.0.1:9090'
var ros_url = 'ws://localhost:9090'
round_topic = null
ros = new ROSLIB.Ros({
  url : ros_url
});
connected_flag = false


ros.on('connection', function() {
  console.log('Connected to websocket server.'); 
  $('#status_circle').addClass('circle-okay') 

  //publishing the round information to the round topic
  round_topic = new ROSLIB.Topic({
    ros:ros,
    name: "/study_round",
    messageType: "bystander_intervention_study/LearnStatus"
  })

});

round_mapping = {
  "level_1":0,
  "level_2":1,
  "level_3":2,
}

function publish_round_information(current_level, playerID, stage, round_num, last_answer){
  return new Promise(function(resolve, reject){
    if(round_topic != null){

      //check if undefined
      round_number = (round_num !== undefined)?round_num:-1
      last_answer = (last_answer !== undefined)?last_answer:-1

      obj = {
        'level':current_level,
        'player':playerID,
        'stage':stage,
        'round_number':round_number,
        'last_answer':last_answer
      }
      round_topic.publish(new ROSLIB.Message(obj))
      resolve()      
    }
    else{
      reject()
    }
  })
}

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
  $('#status_circle').addClass('circle-error')   
});

ros.on('close', function() {

});