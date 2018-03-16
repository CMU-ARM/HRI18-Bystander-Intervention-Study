$('document').ready(function () {

  var score = 0
  var cur_id = 0
  var progress = 0
  var data = {}
  var last_guess = true
  var delay = 2000
  var ANSWER_TIME = 15 //how many seconds they have to answer a question

  function display_question(character, answer, id) {
    //display the character and answers
    $('.question').text(character);
    $('.truth').text(answer);
    //display the number with hint of being wrong
    if (hints.indexOf(cur_id) != -1) {
      $('#numbers').text(". " + (cur_id + 1))
    }
    else {
      $('#numbers').text("  " + (cur_id + 1))
    }
  }



  $.getJSON('/get_json?level=' + cur_level + '&player_id=' + player_id, function (level_file) {

    data = level_file.text
    hints = level_file.hint
    test_sequence = level_file.test_sequence[player_id]
    //rearrage data accordfile.test_sequence[player_id]
    //rearrage data according to the test sequence
    new_data = []
    for (var i = 0; i < test_sequence.length; i++) {
      new_data.push(data[test_sequence[i]])
    }
    data = new_data
    total = data.length;
    send_status("test:guessing")
    $('#main-section').append('<div class="col-sm-12 question question-test">' + data[cur_id].character + '</div>');
    display_question(data[cur_id].character, data[cur_id].meaning, cur_id) //display the questions

    var timer = createTimer($, '#timer', ANSWER_TIME * 1000, function () {
      submit_answer()
      //fire callback
    })



    function send_status(stage) {

      //this function send status to server
      cur_status = {
        "stage": stage,
        "level": cur_level,
        "total": total,
        "index": cur_id,
        "score": score,
        "last_guess": last_guess,
        "player_id": player_id
      }
      $.post('user_status', cur_status)

      function publish_round_info() {
        publish_round_information(cur_level, player_id, 1, cur_id, (last_guess) ? 1 : 0).then(function () {

        }, function () {
          //this means the system isn't ready yet
          //set timout to repeat this ever second
          setTimeout(publish_round_info, 1000)
        })
      }
      publish_round_info()

      //publish_round_information(cur_level, player_id, 1)
    }


    //disable submit button if input is empty
    $('#answer').on('keyup blur', function () {
      if ($('#answer').val().trim().length < 1) {
        $('#btn-done').prop('disabled', true)
      } else {
        $('#btn-done').removeProp('disabled')
      }
    })


    trial_times = 0
    send_goal_start_time = 0
    var goal = null
    function send_actionlib_goal(data, callback) {

      if (trial_times > 10) {
        return callback()
      }
      console.log("sending get request")
      $.get("http://GS15619.SP.CS.CMU.EDU:5000/empathy_callback/" + player_id + "/" + data, "", function (result) {
        console.log(result)
        if (!result.response) {
          trial_times += 1
          return send_actionlib_goal(data, callback)
        }

        diff = (new Date()).getTime() - send_goal_start_time
        if (diff < delay) {
          return setTimeout(callback, delay - diff)
        }
        return callback()
      })

    }

    function submit_answer() {
      var user_answer = $('#answer').val().toLowerCase().trim();
      var correct_answer = data[cur_id].meaning
      $('#btn-next').removeClass("hidden");
      $('#btn-done').addClass("hidden");
      cur_id = cur_id + 1;
      progress = (cur_id / total) * 100;
      $('.progress-bar').attr('style', 'width:' + progress + '%')
      $("input").prop('disabled', true);

      //delay
      $('#btn-next').button('loading');
      $("#result_row").remove()

      if (user_answer == correct_answer.toLowerCase().trim()) {
        last_guess = true
        $('#form-group').addClass("has-success")
        $("#result_bar").append('<div class="alert alert-success result" id="result_row" role="alert">Correct</div>')
        //increment the score
        score += 1
      }
      else {
        last_guess = false
        $('#form-group').addClass("has-failure")
        $("#result_bar").append('<div class="alert alert-danger result" id="result_row" role="alert">Incorrect. The correct answer is "' + correct_answer + '"</div>')
      }
      //send status
      send_status("test:guessed")
      //publish whether the person got it correct or not
      msg_data = last_guess ? 'correct' : 'wrong'
      trial_times = 0
      send_goal_start_time = (new Date()).getTime()
      send_actionlib_goal(msg_data, function () {
        $('#btn-next').button('reset');
      })
    }

    //click submit button
    $('#btn-done').click(function (event) {
      event.preventDefault();
      clearInterval(timer)
      submit_answer()
    })

    function nxtalert(txt, nxt) {
      $.confirm({
        title: 'Message',
        content: txt,
        buttons: {
          next: {
            text: "Next",
            action: nxt
          }
        }
      })
    }

    //click next button 
    $('#btn-next').click(function (event) {
      event.preventDefault();
      $("input").prop('disabled', false);
      if (cur_id == total) {
        //calculate the total score
        var final_score = Math.round(score / total * 100);
        //tell the user what score they get?
        nxtalert('Your score is ' + final_score, function () {

          //check which player it is and do appropriate action
          if (player_id == 1) {
            nxtalert('Please pass to player 2', function () {
              window.location.replace('test?level=' + cur_level + '&player_id=2')
            })
          }
          else if (cur_level == 2 || cur_level == 4) {
            nxtalert('Please pass back to player 1', function () {
              next_level = cur_level + 1
              window.location.replace("learn?level=" + next_level);
            })
            next_level = cur_level + 1
          }
          else if (cur_level <= 4) {
            nxtalert('proceeding to next level', function () {
              next_level = cur_level + 1
              window.location.replace("learn?level=" + next_level);
            })

          }
          else {
            //end of study, start the kill switch
            $.get("http://GS15619.SP.CS.CMU.EDU:5000/kill_switch")
            send_status("test:end_of_study")
            $('#btn-next').prop('disabled', true)
            nxtalert('End of study, please get the experimenter!', function () {
            
            })
          }

        })
      }
      else {
        //update character and meaning
        display_question(data[cur_id].character, data[cur_id].meaning, cur_id)

        //update button and result alert
        $('#btn-next').addClass("hidden");
        $('#result_row').addClass("hidden");
        $('#btn-done').removeClass("hidden");

        //reset input box
        $('#answer').val('');
        $('#form-group').removeClass("has-failure").removeClass("has-success");

        send_status("test:guessing")
        //reset timer
        timer = createTimer($, '#timer', ANSWER_TIME * 1000, function () {
          //fire callback
          submit_answer()
        })
      }
    })
  })
})