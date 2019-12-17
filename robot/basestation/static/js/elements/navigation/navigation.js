$(document).ready(() => { 
  // setup a subscriber for the rover_position topic
  let rover_position_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'rover_position',
    messageType: 'geometry_msgs/Point'
  })
  rover_position_listener.subscribe(function (message) {
    $('#rover-latitude').text(message.x)
    $('#rover-longitude').text(message.y)
    $('#rover-heading').text(message.z)
  })
  // setup a subscriber for the antenna_goal topic
  let antenna_goal_listener = new ROSLIB.Topic({
    name: 'antenna_goal',
    messageType: 'geometry_msgs/Point',
    ros: ros
  })
  antenna_goal_listener.subscribe(function (message) {
    $('#recommended-antenna-angle').text(parseFloat(message.x).toFixed(3))
    $('#distance-to-rover').text(parseFloat(message.y).toFixed(2))
  })
  // setup gps parameters for antenna directing
  let antenna_latitude = new ROSLIB.Param({
    ros: ros,
    name: 'antenna_latitude'
  })
  let antenna_longitude = new ROSLIB.Param({
    ros: ros,
    name: 'antenna_longitude'
  })
  let antenna_start_dir = new ROSLIB.Param({
    ros: ros,
    name: 'antenna_start_dir'
  })

  // setup a subscriber for the rover_goal topic
  let rover_goal_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'rover_goal',
    messageType: 'geometry_msgs/Point'
  })
  rover_goal_listener.subscribe(function (message) {
    $('#recommended-rover-heading').text(parseFloat(message.x).toFixed(3))
    $('#distance-to-goal').text(parseFloat(message.y).toFixed(2))
  })
  // setup gps parameters for rover goals
  let goal_latitude = new ROSLIB.Param({
    ros: ros,
    name: 'goal_latitude'
  })
  let goal_longitude = new ROSLIB.Param({
    ros: ros,
    name: 'goal_longitude'
  })

  function initNavigationPanel () {
    let hasAntennaParams = true
    antenna_latitude.get(function (val) {
      if (val != null) {
        $('#antenna-latitude').text(val)
        antenna_longitude.get(function (val) {
          if (val != null) {
            $('#antenna-longitude').text(val)
            antenna_start_dir.get(function (val) {
              if (val != null) {
                $('#antenna-start-dir').text(val)
                appendToConsole(
                  'Antenna goal parameters already set, displaying antenna directions'
                )
                $('#antenna-inputs').hide()
                $('#antenna-unchanging').show()
              } else {
                appendToConsole(
                  'One or more antenna parameters is missing, if you would like antenna directions then please input them'
                )
                $('#antenna-inputs').show()
                $('#antenna-unchanging').hide()
              }
            })
          } else {
            appendToConsole(
              'One or more antenna parameters is missing, if you would like antenna directions then please input them'
            )
            $('#antenna-inputs').show()
            $('#antenna-unchanging').hide()
          }
        })
      } else {
        appendToConsole(
          'One or more antenna parameters is missing, if you would like antenna directions then please input them'
        )
        $('#antenna-inputs').show()
        $('#antenna-unchanging').hide()
      }
    })

    goal_latitude.get(function (val) {
      if (val != null) {
        $('#goal-latitude').text(val)
        goal_longitude.get(function (val) {
          if (val != null) {
            appendToConsole(
              'GPS goal parameters already set, displaying directions to the goal'
            )
            $('#goal-longitude').text(val)
            $('#goal-inputs').hide()
            $('#goal-unchanging').show()
          } else {
            appendToConsole(
              'One or more GPS goal parameters is missing, if you would like directions to the goal then please input them'
            )
            $('#goal-inputs').show()
            $('#goal-unchanging').hide()
          }
        })
      } else {
        appendToConsole(
          'One or more GPS goal parameters is missing, if you would like directions to the goal then please input them'
        )
        $('#goal-inputs').show()
        $('#goal-unchanging').hide()
      }
    })
  }
  //theres probably a better place to put this, this is just a temporary fix
  initNavigationPanel();
  
  $('#send-antenna-data-btn').on('click', function (event) {
    event.preventDefault()
    let goodInput = true
    if (!$('#antenna-latitude-input').val()) {
      appendToConsole('latitude field empty!')
      goodInput = false
    }
    if (!$('#antenna-longitude-input').val()) {
      appendToConsole('longitude field empty!')
      goodInput = false
    }
    if (!$('#antenna-start-dir-input').val()) {
      appendToConsole('bearing field empty!')
      goodInput = false
    }
    if (goodInput) {
      let initialLatitude = $('#antenna-latitude-input').val()
      let initialLongitude = $('#antenna-longitude-input').val()
      let initialBearing = $('#antenna-start-dir-input').val()
      antenna_latitude.set(parseFloat(initialLatitude))
      antenna_longitude.set(parseFloat(initialLongitude))
      antenna_start_dir.set(parseFloat(initialBearing))
      $('#antenna-latitude').text(initialLatitude)
      $('#antenna-longitude').text(initialLongitude)
      $('#antenna-start-dir').text(initialBearing)
      $('#antenna-inputs').hide()
      $('#antenna-unchanging').show()
    }
  })
  $('#change-antenna-data-btn').on('click', function (event) {
    event.preventDefault()
    $('#antenna-inputs').show()
    $('#antenna-unchanging').hide()
  })

  $('#send-goal-pos-btn').on('click', function (event) {
    event.preventDefault()
    let goodInput = true
    if (!$('#goal-latitude-input').val()) {
      appendToConsole('latitude field empty!')
      goodInput = false
    }
    if (!$('#goal-longitude-input').val()) {
      appendToConsole('longitude field empty!')
      goodInput = false
    }
    if (goodInput) {
      let desiredLatitude = $('#goal-latitude-input').val()
      let desiredLongitude = $('#goal-longitude-input').val()
      goal_latitude.set(parseFloat(desiredLatitude))
      goal_longitude.set(parseFloat(desiredLongitude))
      $('#goal-latitude').text(desiredLatitude)
      $('#goal-longitude').text(desiredLongitude)
      $('#goal-inputs').hide()
      $('#goal-unchanging').show()
    }
  })

  $('#change-goal-pos-btn').on('click', function (event) {
    event.preventDefault()
    $('#goal-inputs').show()
    $('#goal-unchanging').hide()
  }) 

  
var count = 0;


$('#antenna-input-lat-format-btn').on('click',function(event){

  CreateAntennaLatitudeHandler();
  CreateAntennaLongtitudeHandler();
  //CreateAntennaButtons();


})


 $('#antenna-inputs-confirm-btn').on('click', function(event){
  appendToConsole('balls');
    let src = `
      <ul class = "list-group" id = "antenna-stats-list">
        <li class = "list-group-item"> <label for = "" > Latitude</label>  <span class="float-right">Right aligned </span >  </li>
        <li class = "list-group-item">  <label for = "" > Longtitude</label>  <span class="float-right">Right aligned </span ></li>
        <li class = "list-group-item"> <label for = "" > Heading</label>  <span class="float-right">Right aligned </span >  </li>
        <li class = "list-group-item">  <label for = "" > Speed</label>  <span class="float-right">Right aligned </span ></li>
        <li class = "list-group-item">  <label for = "" > Angular Velocity</label>  <span class="float-right">Right aligned </span ></li>
                


        
      </ul>
    `;
    $('#data-start').prepend(src);
    $('#bullshit').text('Did it work?');
  })
  


function CreateAntennaLatitudeHandler(){
  $('#antenna-lat-decimal-degrees-btn').on('click', function (event) {
    
    let src = `
   
             <span class="input-group-text">Latitude </span>
           <input type="text" class="form-control" placeholder="Decimal">
           <span class="input-group-text">.</span>
           <input type="text" class="form-control" placeholder="Degrees">
           <span class="input-group-text">°</span>
          
         
  <hr>
  `
  event.preventDefault();
  $("#antenna-input-lat-format-btn").dropdown('toggle');
  $('#antenna-input-lat-format-btn').detach();

  $("#antenna-lat-input-group").append(src);


  })
  
$('#antenna-lat-degrees-decimal-minutes-btn').on('click', function (event) {
    
    let src = `
   
           <span class="input-group-text">Latitude </span>
           
           <input type="text" class="form-control" placeholder="Degrees">
           <span class="input-group-text">°</span>
          
           <input type="text" class="form-control" placeholder="Decimal">
           <span class="input-group-text">.</span>
          
           <input type="text" class="form-control" placeholder="Mins">
           <span class="input-group-text">'</span>
  
  <hr>
  `
  event.preventDefault();
  $("#antenna-input-lat-format-btn").dropdown('toggle');
  $('#antenna-input-lat-format-btn').detach();

  $("#antenna-lat-input-group").append(src);


  })
$('#antenna-lat-degrees-minutes-seconds-btn').on('click', function (event) {
    
    let src = `
   
           <span class="input-group-text">Latitude </span>

           <input type="text" class="form-control" placeholder="Degrees">
           <span class="input-group-text">°</span>

           <input type="text" class="form-control" placeholder="Mins">
           <span class="input-group-text">'</span>
           
           <input type="text" class="form-control" placeholder="Seconds">
           <span class="input-group-text">"</span>
          
         
  <hr>
  `
  event.preventDefault();
  $("#antenna-input-lat-format-btn").dropdown('toggle');
  $('#antenna-input-lat-format-btn').detach();

  $("#antenna-lat-input-group").append(src);


  })
}
function CreateAntennaLongtitudeHandler(){
  $('#antenna-long-decimal-degrees-btn').on('click', function (event) {
    

    let src = `
             <span class="input-group-text">Latitude </span>
           <input type="text" class="form-control" placeholder="Decimal">
           <span class="input-group-text">.</span>
           <input type="text" class="form-control" placeholder="Degrees">
           <span class="input-group-text">°</span>
  `
  event.preventDefault();
  $("#antenna-input-long-format-btn").dropdown('toggle');
  $('#antenna-input-long-format-btn').detach();
  $("#antenna-long-input-group").append(src);


  })
  $('antenna-long-degrees-decimal-minutes-btn').on('click', function (event) {
    

    let src = `
          <span class="input-group-text">Latitude </span>
           <input type="text" class="form-control" placeholder="Degrees">
        
           <span class="input-group-text">°</span>
           <input type="text" class="form-control" placeholder="Decimal">
           <span class="input-group-text">.</span>
           <input type="text" class="form-control" placeholder="Mins">
           <span class="input-group-text">'</span>
  `
  event.preventDefault();
  $("#antenna-input-long-format-btn").dropdown('toggle');
  $('#antenna-input-long-format-btn').detach();
  $("#antenna-long-input-group").append(src);


  })
  $('#antenna-long-degrees-minutes-seconds-btn').on('click', function (event) {
    

    let src = `
           <span class="input-group-text">Latitude </span>

           <input type="text" class="form-control" placeholder="Degrees">
           <span class="input-group-text">°</span>

           <input type="text" class="form-control" placeholder="Mins">
           <span class="input-group-text">'</span>
           
           <input type="text" class="form-control" placeholder="Seconds">
           <span class="input-group-text">"</span>
  `
  event.preventDefault();
  $("#antenna-input-long-format-btn").dropdown('toggle');
  $('#antenna-input-long-format-btn').detach();
  $("#antenna-long-input-group").append(src);


  })
} 

  
  $("#new-goal-coordinates-btn").on('click', function(event) {
    let src = `
    <div id = "goal-lat-input-group-` + count + `" class = "input-group">

        <div class = "dropdown" >
                  <button type="button" id = "goal-lat-select-format-btn-` + count + `" class="btn btn-primary dropdown-toggle btn-sm" data-toggle = "dropdown">Select Latitude Layout</button>
                  <span class = "caret"> </span>
                  <ul class = "dropdown-menu">
                    <li class = "dropdown-header"> Select </li>
                    <li><button id = "goal-lat-decimal-degrees-btn-` + count + `" class = "btn btn-primary btn-sm" value = ""> Decimal Degrees </button> </li>
                    <li><button id = "goal-lat-degrees-decimal-minutes-btn-` + count + `" class = "btn btn-primary btn-sm"> Degrees Decimal Minutes </button> </li>
                    <li><button id = "goal-lat-degrees-minutes-seconds-btn-` + count + `" class = "btn btn-primary btn-sm"> Degrees Minutes Seconds</button> </li>
                    </ul>
                    
        </div>
    </div>

      <div id = "goal-long-input-group-` + count + `" class = "input-group">

         <div class = "dropdown" >
                  <button type="button" id = "goal-long-select-format-btn-` + count + `" class="btn btn-primary dropdown-toggle btn-sm" data-toggle = "dropdown">Select Longtitude Layout</button>
                  <span class = "caret"> </span>
                  <ul class = "dropdown-menu">
                    <li class = "dropdown-header"> Select </li>
                    <li><button id = "goal-long-decimal-degrees-btn-` + count + `" class = "btn btn-primary btn-sm" value = ""> Decimal Degrees </button> </li>
                    <li><button id = "goal-long-degrees-decimal-minutes-btn-` + count + `" class = "btn btn-primary btn-sm"> Degrees Decimal Minutes </button> </li>
                    <li><button id = "goal-long-degrees-minutes-seconds-btn-` + count + `" class = "btn btn-primary btn-sm"> Degrees Minutes Seconds</button> </li>
                    </ul>
                    
        </div>
        
    </div>
    <div id = "buttons-input-group-` + count + `" class = "input-group">
    </div>
      <hr>
    `
    
    
    $("#goal-modal-body").append(src);
    CreateGoalLatitudeHandler(count);
    CreateGoalLongtitudeHandler(count);
    CreateButtons(count);
    count = count+1;
   
  })




  function CreateGoalLatitudeHandler(current){
    $("#goal-lat-decimal-degrees-btn-" + current).on('click', function(event){
     let src = `
            <span class="input-group-text">Latitude </span>
           <input type="text" class="form-control" placeholder="Decimal">
           <span class="input-group-text">.</span>
           <input type="text" class="form-control" placeholder="Degrees">
           <span class="input-group-text">°</span>
      `;
      $('#goal-lat-select-format-btn-' + current).dropdown('toggle');
      

      $('#goal-lat-select-format-btn-' + current).detach();
      $('#goal-lat-input-group-' + current).append(src);

    })

     $("#goal-lat-degrees-decimal-minutes-btn-" + current).on('click', function(event){
     let src = `
           <span class="input-group-text">Latitude </span>
           <input type="text" class="form-control" placeholder="Degrees">
        
           <span class="input-group-text">°</span>
           <input type="text" class="form-control" placeholder="Decimal">
           <span class="input-group-text">.</span>
           <input type="text" class="form-control" placeholder="Mins">
           <span class="input-group-text">'</span>
      `;
      $('#goal-lat-select-format-btn-' + current).dropdown('toggle');
      

      $('#goal-lat-select-format-btn-' + current).detach();
      $('#goal-lat-input-group-' + current).append(src);

    })
     $("#goal-lat-degrees-minutes-seconds-btn-" + current).on('click', function(event){
     let src = `
           <span class="input-group-text">Latitude </span>

           <input type="text" class="form-control" placeholder="Degrees">
           <span class="input-group-text">°</span>

           <input type="text" class="form-control" placeholder="Mins">
           <span class="input-group-text">'</span>
           
           <input type="text" class="form-control" placeholder="Seconds">
           <span class="input-group-text">"</span>
      `;

      $('#goal-lat-select-format-btn-' + current).dropdown('toggle');
      

      $('#goal-lat-select-format-btn-' + current).detach();
      $('#goal-lat-input-group-' + current).append(src);

    })
  }
    function CreateGoalLongtitudeHandler(current){
    $("#goal-long-decimal-degrees-btn-" + current).on('click', function(event){
      
      let src = `
           <span class="input-group-text">Longtitude' </span>
           <input type="text" class="form-control" placeholder="Decimal">
           <span class="input-group-text">.</span>
           <input type="text" class="form-control" placeholder="Degrees">
           <span class="input-group-text">°</span>
      `;
         $('#goal-long-select-format-btn-' + current).dropdown('toggle');
      $('#goal-long-select-format-btn-' + current).detach();
      $('#goal-long-input-group-' + current).append(src);

    })

    $("#goal-long-degrees-decimal-minutes-btn-" + current).on('click', function(event){
      
      let src = `
        <span class="input-group-text">Longtitude </span>
        <input type="text" class="form-control" placeholder="Degrees">
        
           <span class="input-group-text">°</span>
           <input type="text" class="form-control" placeholder="Decimal">
           <span class="input-group-text">.</span>
           <input type="text" class="form-control" placeholder="Mins">
           <span class="input-group-text">'</span>
      `;
         $('#goal-long-select-format-btn-' + current).dropdown('toggle');
      $('#goal-long-select-format-btn-' + current).detach();
      $('#goal-long-input-group-' + current).append(src);
 
    })
    $("#goal-long-degrees-minutes-seconds-btn-" + current).on('click', function(event){
      
      let src = `
        <span class="input-group-text">Longtitude </span>
        <input type="text" class="form-control" placeholder="Degrees">
        
           <span class="input-group-text">°</span>
           <input type="text" class="form-control" placeholder="Mins">
           <span class="input-group-text">'</span>
           <input type="text" class="form-control" placeholder="Seconds">
           <span class="input-group-text">"</span>
      `;

      $('#goal-long-select-format-btn-' + current).dropdown('toggle');
      $('#goal-long-select-format-btn-' + current).detach();
      $('#goal-long-input-group-' + current).append(src);
 
    })
     

    }
    function CreateButtons(current){
      let src = `

         <button id = "confirm-btn-` + current + `" type = "button" class = "btn btn-success"> Confirm </button>
          <button id = "change-btn-` + current + `" type = "button" class = "btn btn-warning " disabled> Change </button>
          <button id = "delete-btn-` + current + `" type = "button" class = "btn btn-danger "> Delete </button>
               
      `
      $("#buttons-input-group-" + current).append(src);
      CreateDeleteButtonHandler(current);

    }
    function CreateConfirmButtonHandler(current){
  $('#confirm-btn-' + current).on('click' , function(event){
  appendToConsole(current  + ' confirm was clikced')
  $('#change-btn-' + current).prop('disabled', false);
  $('#confirm-btn-' + current).prop('disabled', true);
    
})
}
function CreateChangeButtonHandler(current) {
   $('#change-btn-' + current).on('click' , function(event){
  
  
  $('#change-btn-' + current).prop('disabled', false);
  $('#confirm-btn-' + current).prop('disabled', false);  
})
}
function CreateDeleteButtonHandler(current) {

   $('#delete-btn-' + current).on('click' , function(event){
    appendToConsole(current + ' delete was clicked')

    $('#goal-lat-input-group-' + current).remove();
    $('#goal-long-input-group-' + current).remove();
    $('#buttons-input-group-' + current).remove();
  

  count = count-1;
  
})  
}
})

