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
    //$('#recommended-antenna-angle').text(parseFloat(message.x).toFixed(3))
    //$('#distance-to-rover').text(parseFloat(message.y).toFixed(2))
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
    //$('#recommended-rover-heading').text(parseFloat(message.x).toFixed(3))
    //$('#distance-to-goal').text(parseFloat(message.y).toFixed(2))
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
  
  /*$('#send-antenna-data-btn').on('click', function (event) {
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
*/
function NavigationQueue() {
  this.data = [];
  this.enqueue = function(item){
    this.data.push(item);
  }
  this.dequeue = function(){
    this.data.shift();
  }
  
}

var navQueue = new NavigationQueue();
var count = 0;



$('#antenna-select-lat-format-btn').one('click',function(event){
  event.preventDefault();
  CreateAntennaLatitudeHandler();
  //CreateAntennaButtons();


})                
  $('#antenna-select-long-format-btn').one('click',function(event){
  event.preventDefault();
    CreateAntennaLongtitudeHandler();


})           


 $('#antenna-confirm-btn').on('click', function(event){
    //check if the inputs are valid
         let lat = -1;
          let long = -1;

    let src = `
    <hr>
      <ul class = "list-group" id = "antenna-stats-list"> 
        <li class = "list-group-item"> <label for = "" > Antenna Latitude</label>  <span id = "antenna-lat" class="float-right">Right aligned </span >  </li>
        <li class = "list-group-item">  <label for = "" > Antenna Longtitude</label>  <span id = "antenna-long" class="float-right">Right aligned </span ></li>
        <li class = "list-group-item"> <label for = "" > Antenna Heading</label>  <span id = "antenna-heading" class="float-right">Right aligned </span >  </li>
                
      </ul>
    `;
      
      try{
        if($("#antenna-lat-decimal-degrees-btn").data('clicked') ) {

           let decdeg = parseFloat( ($("#antenna-lat-DD-dec-deg-input").val()) ) ;
           if(isNaN(decdeg)) throw "Bad Input lat DD";
           lat  = decdeg;
        }
        else if($("#antenna-lat-degrees-decimal-minutes-btn").data('clicked') ){
            let dec = parseFloat($('#antenna-lat-DDM-deg-input').val());
            let min = parseFloat($('#antenna-lat-DDM-dec-min-input').val());
            if( isNaN(dec) || isNaN (min)) throw "Bad input lat DDM";

            lat = dec+ (min/60);
                    
        } 
        else if($("#antenna-lat-degrees-minutes-seconds-btn").data('clicked') ){

            let dec = parseFloat($('#goal-lat-DMS-deg-input').val());
            let mins = parseFloat($('#goal-lat-DMS-min-input').val());
            let secs = parseFloat($('#goal-lat-DMS-sec-input').val());
            if( isNaN(dec) || isNaN(mins) || isNaN(secs)) throw "Bad input lat DMS";

            lat = dec + (mins/60 + secs/3600);

        }  

        if($("#antenna-long-decimal-degrees-btn").data('clicked') ) {
          let decdeg = parseFloat( ($("#antenna-long-DD-dec-deg-input").val()) ) ;
           if(isNaN(decdeg)) throw "Bad Input long DD";
           long  = decdeg;
        }
        else if($("#antenna-long-degrees-decimal-minutes-btn").data('clicked') ){
            let dec = parseFloat($('#antenna-long-DDM-deg-input').val());
            let min = parseFloat($('#antenna-long-DDM-dec-min-input').val());
            if( isNaN(dec) || isNaN (min)) throw "Bad input long DDM";

            long = dec+ (min/60);
        } 
        else if($("#antenna-long-degrees-minutes-seconds-btn").data('clicked') ){
              let dec = parseFloat($('#goal-long-DMS-deg-input').val());
            let mins = parseFloat($('#goal-long-DMS-min-input').val());
            let secs = parseFloat($('#goal-long-DMS-sec-input').val());
            if( isNaN(dec) || isNaN(mins) || isNaN(secs)) throw "Bad input long DMS";

            logn = dec + (mins/60 + secs/3600);
        }  
        


          $('#antenna-stats-list').append(src);
         //disable text fields until the user wants to change them
          $('#antenna-fieldset').prop('disabled',true);
          $('#antenna-confirm-btn').prop('disabled', true);
      }
        catch(e){
            appendToConsole(e);
        }


})
     
    $('#antenna-change-btn').on('click', function(event){
      //need to kill the children of the list, or else i wont the list wil be lost and ill lose access to it forever.
    $('#antenna-stats-list').empty();
    $('#antenna-fieldset').prop('disabled',false);
    $('#antenna-confirm-btn').prop('disabled', false);
    $('#antenna-change-btn').prop('disabled',false);

 })

function CreateAntennaLatitudeHandler(){
  $('#antenna-lat-decimal-degrees-btn').on('click', function (event) {
    $(this).data('clicked', true);

    let src = `
   
          <span class="input-group-text font-weight-bold" >Latitude </span>
          <input id = "antenna-lat-DD-dec-deg-input" type="text" class="form-control" placeholder="Decimal degrees">
          
         
  <hr>
  `
  event.preventDefault();
  $("#antenna-select-lat-format-btn").dropdown('toggle');
  $('#antenna-select-lat-format-btn').detach();

  $("#antenna-lat-input-group").append(src);


  })
  
$('#antenna-lat-degrees-decimal-minutes-btn').on('click', function (event) {
    $(this).data('clicked', true);
    let src = `
   
           <span class="input-group-text font-weight-bold">Latitude </span>
           
           <input id = "antenna-lat-DDM-deg-input" type="text" class="form-control" placeholder="Degrees">
           <span class="input-group-text">°</span>
          
           <input id = "antenna-lat-DDM-dec-min-input" type="text" class="form-control" placeholder="Decimal Mins">
           <span class="input-group-text">'</span>
  
  <hr>
  `
  event.preventDefault();
  $("#antenna-select-lat-format-btn").dropdown('toggle');
  $('#antenna-select-lat-format-btn').detach();

  $("#antenna-lat-input-group").append(src);


  })
$('#antenna-lat-degrees-minutes-seconds-btn').on('click', function (event) {
    $(this).data('clicked', true);
    let src = `
   
           <span class="input-group-text">Latitude </span>

           <input id = "antenna-lat-DMS-deg-input" type="text" class="form-control" placeholder="Degrees">
           <span class="input-group-text">°</span>

           <input id = "antenna-lat-DMS-min-input" type="text" class="form-control" placeholder="Mins">
           <span class="input-group-text">'</span>
           
           <input id = "antenna-lat-DMS-sec-input" type="text" class="form-control" placeholder="Seconds">
           <span class="input-group-text">"</span>
          
         
  <hr>
  `
  event.preventDefault();
  $("#antenna-select-lat-format-btn").dropdown('toggle');
  $('#antenna-select-lat-format-btn').detach();

  $("#antenna-lat-input-group").append(src);


  })
}

function CreateAntennaLongtitudeHandler(){
  $('#antenna-long-decimal-degrees-btn').on('click', function (event) {
    $(this).data('clicked', true);

    let src = `
           <span class="input-group-text">Longtitude </span>
           <input id = "antenna-long-DD-dec-deg-input" type="text" class="form-control" placeholder="Decimal Degrees">
           
  `
  event.preventDefault();
  $("#antenna-select-long-format-btn").dropdown('toggle');
  $('#antenna-select-long-format-btn').detach();
  $("#antenna-long-input-group").append(src);


  })
  $('antenna-long-degrees-decimal-minutes-btn').on('click', function (event) {
    $(this).data('clicked', true);

    let src = `
          <span class="input-group-text">Longtitude </span>
           <input id = "antenna-long-DDM-deg-input" type="text" class="form-control" placeholder="Degrees">
        
           <span class="input-group-text">°</span>

           <input id = "antenna-long-DDM-dec-min-input" type="text" class="form-control" placeholder="Mins">
           <span class="input-group-text">'</span>
  `
  event.preventDefault();
  $("#antenna-select-long-format-btn").dropdown('toggle');
  $('#antenna-select-long-format-btn').detach();
  $("#antenna-long-input-group").append(src);


  })
  $('#antenna-long-degrees-minutes-seconds-btn').on('click', function (event) {
    $(this).data('clicked', true);

    let src = `
           <span class="input-group-text">Longtitude </span>

           <input id = "antenna-long-DMS-deg-input" type="text" class="form-control" placeholder="Degrees">
           <span class="input-group-text">°</span>

           <input id = "antenna-long-DMS-min-input" type="text" class="form-control" placeholder="Mins">
           <span class="input-group-text">'</span>
           
           <input id = "antenna-long-DMS-sec-input" type="text" class="form-control" placeholder="Seconds">
           <span class="input-group-text">"</span>
  `
  event.preventDefault();
  $("#antenna-select-long-format-btn").dropdown('toggle');
  $('#antenna-select-long-format-btn').detach();
  $("#antenna-long-input-group").append(src);


  })
} 


  $("#new-goal-coordinates-btn").on('click', function(event) {
    let src = `
    <fieldset id = "goal-lat-fieldset-`+count + `">
           
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
    </fieldset>
    <div id = "buttons-input-group-` + count + `" class = "input-group">
    </div>
      <hr>
    `
    
    
    $("#goal-modal-body").append(src);
    CreateGoalLatitudeHandler(count);
    CreateGoalLongtitudeHandler(count);
    CreateGoalButtons(count);
    
    count = count+1;
   
  })
  function InsertDataInQueue(current,lat_format,long_format){
    
    let lat = -1;
    let long = -1;
    if(lat_format == 0){
      //decimal-degrees mode
      lat = parseFloat($('#goal-lat-DD-dec-deg-input-'+current).val());
   
    }
    else if(lat_format == 1){
      //degrees-decimal-minutes mode
      let dec = parseFloat($('#goal-lat-DDM-deg-input-'+current).val());
      let min = parseFloat($('#goal-lat-DDM-dec-min-input-'+current).val());
      lat = dec+ (min/60);

    }
    else if(lat_format == 2){
      //degress-minutes-seconds mode
      let dec = parseFloat($('#goal-lat-DMS-deg-input-'  + current).val());
      let mins = parseFloat($('#goal-lat-DMS-min-input-' + current).val());
      let secs = parseFloat($('#goal-lat-DMS-sec-input-' + current).val());

      lat = dec + (mins/60 + secs/3600);
    }
    if(long_format == 0){
      //decimal-degrees mode
      long = parseFloat($('#goal-long-DD-dec-deg-input-'+current).val());
   
    }
    else if(long_format == 1){
      //degrees-decimal-minutes mode
      let dec = parseFloat($('#goal-long-DDM-deg-input-'+current).val());
      let min = parseFloat($('#goal-long-DDM-dec-min-input-'+current).val());
      long = dec+ (min/60);

    }
    else if(long_format == 2){
      //degress-minutes-seconds mode
      let dec = parseFloat($('#goal-long-DMS-deg-input-'  + current).val());
      let mins = parseFloat($('#goal-long-DMS-min-input-' + current).val());
      let secs = parseFloat($('#goal-long-DMS-sec-input-' + current).val());

      long = dec + (mins/60 + secs/3600);
    }
    
    let latlongpair = {
      latitude : lat,
      longitude : long
    }
    navQueue.enqueue(latlongpair);
    console.log(navQueue);
  }   



  function CreateGoalLatitudeHandler(current){

    $("#goal-lat-decimal-degrees-btn-" + current).on('click', function(event){
      $(this).data('clicked', true);
     let src = `
           <span class="input-group-text">Latitude </span>
           <input id = "goal-lat-DD-dec-deg-input-`+current+`" type="text" class="form-control" placeholder="Decimal Degrees">
           
           

      `;
      $('#goal-lat-select-format-btn-' + current).dropdown('toggle');
      

      $('#goal-lat-select-format-btn-' + current).detach();
      $('#goal-lat-input-group-' + current).append(src);

      })

     $("#goal-lat-degrees-decimal-minutes-btn-" + current).on('click', function(event){
      $(this).data('clicked', true);
     let src = `
           <span class="input-group-text">Latitude </span>
           <input id = "goal-lat-DDM-deg-input-`+current+`" type="text" class="form-control" placeholder="Degrees">
        
           <span class="input-group-text">°</span>
           

           <input id = "goal-lat-DDM-dec-min-input-` + current + `" type="text" class="form-control" placeholder="Decimal-Mins">
           <span class="input-group-text">'</span>
           
      `;
      $('#goal-lat-select-format-btn-' + current).dropdown('toggle');
      

      $('#goal-lat-select-format-btn-' + current).detach();
      $('#goal-lat-input-group-' + current).append(src);


    })
     $("#goal-lat-degrees-minutes-seconds-btn-" + current).on('click', function(event){
      $(this).data('clicked', true);
     let src = `
           <span class="input-group-text">Latitude </span>

            <input id = "goal-lat-DMS-deg-input-`+current+`" type="text" class="form-control" placeholder="Degrees">
           <span class="input-group-text">°</span>

           <input id = "goal-lat-DMS-min-input-` + current + `" type="text" class="form-control" placeholder="Mins">
           <span class="input-group-text">'</span>
           
           <input id = "goal-lat-DMS-sec-input-` + current + `" type="text" class="form-control" placeholder="Seconds">
           <span class="input-group-text">"</span>
      `;

      $('#goal-lat-select-format-btn-' + current).dropdown('toggle');
      

      $('#goal-lat-select-format-btn-' + current).detach();
      $('#goal-lat-input-group-' + current).append(src);
     
    })
     
  }



    function CreateGoalLongtitudeHandler(current){
    $("#goal-long-decimal-degrees-btn-" + current).on('click', function(event){
      $(this).data('clicked', true);
      
      let src = `
           <span class="input-group-text">Longtitude' </span>
           <input id = "goal-long-DD-dec-deg-input-`+current+`" type="text" class="form-control" placeholder="Decimal Degrees">
           
      `;
         $('#goal-long-select-format-btn-' + current).dropdown('toggle');
      $('#goal-long-select-format-btn-' + current).detach();
      $('#goal-long-input-group-' + current).append(src);

    })

    $("#goal-long-degrees-decimal-minutes-btn-" + current).on('click', function(event){
      $(this).data('clicked', true);
      
      let src = `
        <span class="input-group-text">Longtitude </span>
        <input id = "goal-long-DDM-deg-input-`+current+`" type="text" class="form-control" placeholder="Degrees">
        
           <span class="input-group-text">°</span>

           <input id = "goal-long-DDM-dec-min-input-` + current +`" type="text" class="form-control" placeholder="Decimal Mins">
           <span class="input-group-text">'</span>
      `;
         $('#goal-long-select-format-btn-' + current).dropdown('toggle');
      $('#goal-long-select-format-btn-' + current).detach();
      $('#goal-long-input-group-' + current).append(src);
 
    })
    $("#goal-long-degrees-minutes-seconds-btn-" + current).on('click', function(event){
      $(this).data('clicked', true);
      
      let src = `
        <span class="input-group-text">Longtitude </span>
        <input id = "goal-long-DMS-deg-input-` + current+ `" type="text" class="form-control" placeholder="Degrees">
        
           <span class="input-group-text">°</span>
           <input id = "goal-long-DMS-min-input-` + current+ `" type="text" class="form-control" placeholder="Mins">
           <span class="input-group-text">'</span>
           <input id = "goal-long-DMS-sec-input-` + current+ `"  type="text" class="form-control" placeholder="Seconds">
           <span class="input-group-text">"</span>
      `;

      $('#goal-long-select-format-btn-' + current).dropdown('toggle');
      $('#goal-long-select-format-btn-' + current).detach();
      $('#goal-long-input-group-' + current).append(src);
 
    })
     

    }
    function CreateGoalButtons(current){
      let src = `

         <button id = "confirm-btn-` + current + `" type = "button" class = "btn btn-success"> Confirm </button>
          <button id = "change-btn-` + current + `" type = "button" class = "btn btn-warning " disabled> Change </button>
          <button id = "delete-btn-` + current + `" type = "button" class = "btn btn-danger "> Delete </button>
               
      `
      $("#buttons-input-group-" + current).append(src);
      CreateConfirmButtonHandler(current);
      CreateChangeButtonHandler(current);
      CreateDeleteButtonHandler(current);

    }
    function CreateConfirmButtonHandler(current){
  $('#confirm-btn-' + current).on('click' , function(event){
    let lat_format = -1;
    let long_format = -1;
    if($("#goal-lat-decimal-degrees-btn").data('click'))
      long_format = 0;
    else if($("#goal-lat-degrees-decimal-minutes-btn").data('click'))
      long_format = 1;
    else if($("#goal-lat-degrees-minutes-seconds-btn").data('click'))
      long_format = 2;
    else appendToConsole("error?");

    if($("#goal-long-decimal-degrees-btn").data('click'))
      long_format = 0;
    else if($("#goal-long-degrees-decimal-minutes-btn").data('click'))
      long_format = 1;
    else if($("#goal-long-degrees-minutes-seconds-btn").data('click'))
      long_format = 2;
    else appendToConsole("error?");
    
    //disable text fields
    $('#goal-lat-fieldset-' + current).prop('disabled',true);

  $('#change-btn-' + current).prop('disabled', false);
  $('#confirm-btn-' + current).prop('disabled', true);

   InsertDataInQueue(current,lat_format,long_format);
})    
}
function CreateChangeButtonHandler(current) {
   $('#change-btn-' + current).on('click' , function(event){
  
  
    $('#goal-lat-fieldset-' + current).prop('disabled',false);
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

