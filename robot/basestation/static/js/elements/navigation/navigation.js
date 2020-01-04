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
    $('#antenna-display-rec-angle').text(parseFloat(message.x).toFixed(3))
    $('#antenna-display-dist-to-rover').text(parseFloat(message.y).toFixed(2))
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
    $('#goal-display-rover-heading').text(parseFloat(message.x).toFixed(3))
    $('#goal-display-distance').text(parseFloat(message.y).toFixed(2))
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
    /*
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
    }) */
  }
  //theres probably a better place to put this, this is just a temporary fix
  initNavigationPanel();
  

function NavigationQueue() {
  this.data = [];
  let removed = 0;
  this.enqueue = function(item){
    this.data.push(item);
  }
  this.dequeue = function(){
    this.data.shift();
  }
  this.remove = function(index){
    this.data.splice(index-removed,1);
    removed++;
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

            let dec = parseFloat($('#antenna-lat-DMS-deg-input').val());
            let mins = parseFloat($('#antenna-lat-DMS-min-input').val());
            let secs = parseFloat($('#antenna-lat-DMS-sec-input').val());
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
              let dec = parseFloat($('#antenna-long-DMS-deg-input').val());
            let mins = parseFloat($('#antenna-long-DMS-min-input').val());
            let secs = parseFloat($('#antenna-long-DMS-sec-input').val());
            if( isNaN(dec) || isNaN(mins) || isNaN(secs)) throw "Bad input long DMS";

            long = dec + (mins/60 + secs/3600);
        }  
        let bearing = $("#antenna-bearing-input").val();
        if(isNaN(bearing)) throw "Bad input bearing";

          //ROS params
          antenna_latitude.set(lat);
          antenna_longitude.set(long);


          $.ajax('/navigation/inputTemplates/antenna-stats', {
      success: function (result) {
          
          $('#antenna-stats-list').append(result);  
       }})  

  
         //disable text fields until the user wants to change them
          $('#antenna-fieldset').prop('disabled',true);
          $('#antenna-confirm-btn').prop('disabled', true);

          //update antenna diplayed fields
          $('#antenna-display-lat').text(lat);
          $('#antenna-display-long').text(long);
          $('#antenna-display-heading').text(bearing);
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
      event.preventDefault();

    $(this).data('clicked', true);
    
    $("#antenna-select-lat-format-btn").dropdown('toggle');
    $('#antenna-select-lat-format-btn').detach();


    $.ajax('/navigation/inputTemplates/antenna-DD/'+'lat', {
      success: function (result) {
          $("#antenna-lat-input-group").append(result);

       }})  


  

  })
  
$('#antenna-lat-degrees-decimal-minutes-btn').on('click', function (event) {
    event.preventDefault();

    $(this).data('clicked', true);
    
    $("#antenna-select-lat-format-btn").dropdown('toggle');
    $('#antenna-select-lat-format-btn').detach();


    $.ajax('/navigation/inputTemplates/antenna-DDM/'+'lat', {
      success: function (result) {
          $("#antenna-lat-input-group").append(result);

       }})  




  })
$('#antenna-lat-degrees-minutes-seconds-btn').on('click', function (event) {
     event.preventDefault();

    $(this).data('clicked', true);
    
    $("#antenna-select-lat-format-btn").dropdown('toggle');
    $('#antenna-select-lat-format-btn').detach();


    $.ajax('/navigation/inputTemplates/antenna-DMS/'+'lat', {
      success: function (result) {
          $("#antenna-lat-input-group").append(result);

       }})  

  })
}

function CreateAntennaLongtitudeHandler(){
     $('#antenna-long-decimal-degrees-btn').on('click', function (event) {
          event.preventDefault();
           $(this).data('clicked', true);

     $("#antenna-select-long-format-btn").dropdown('toggle');
     $('#antenna-select-long-format-btn').detach();
 
       $.ajax('/navigation/inputTemplates/antenna-DD/'+'long', {
          success: function (result) {
          $("#antenna-long-input-group").append(result);

       }})  


  })

  $('#antenna-long-degrees-decimal-minutes-btn').on('click', function (event) {

     event.preventDefault();

    $(this).data('clicked', true);
    
    $("#antenna-select-long-format-btn").dropdown('toggle');
    $('#antenna-select-long-format-btn').detach();


    $.ajax('/navigation/inputTemplates/antenna-DDM/'+'long', {
      success: function (result) {
          $("#antenna-long-input-group").append(result);

       }})  



  })
  $('#antenna-long-degrees-minutes-seconds-btn').on('click', function (event) {
   
    event.preventDefault();

    $(this).data('clicked', true);
    
    $("#antenna-select-long-format-btn").dropdown('toggle');
    $('#antenna-select-long-format-btn').detach();


    $.ajax('/navigation/inputTemplates/antenna-DMS/'+'long', {
      success: function (result) {
          $("#antenna-long-input-group").append(result);

       }})  


  })
} 


  $("#new-goal-coordinates-btn").on('click', function(event) {
  
    $.ajax('/navigation/inputTemplates/new-goal-coordinates-btn/'+count, {
      success: function (result) {
        $("#goal-modal-body").append(result);

        CreateGoalLatitudeHandler(count);
        CreateGoalLongtitudeHandler(count);
        CreateGoalButtons(count);
        // UpdateGoalStats();
        count++;    
      }})
  })

  function InsertDataInQueue(current,lat_format,long_format){
    
    let lat = -1;
    let long = -1;
    try{
    if(lat_format == 0){
      //decimal-degrees mode
      lat = parseFloat($('#goal-lat-DD-dec-deg-input-'+current).val());
      if(isNaN(lat)) throw "Bad input lat DD";
    }
    else if(lat_format == 1){
      //degrees-decimal-minutes mode
      let dec = parseFloat($('#goal-lat-DDM-deg-input-'+current).val());
      let min = parseFloat($('#goal-lat-DDM-dec-min-input-'+current).val());
      if(isNaN(dec) || isNaN(min)) throw "Bad input lat DDM"
      lat = dec+ (min/60);

    }
    else if(lat_format == 2){
      //degress-minutes-seconds mode
      let dec = parseFloat($('#goal-lat-DMS-deg-input-'  + current).val());
      let mins = parseFloat($('#goal-lat-DMS-min-input-' + current).val());
      let secs = parseFloat($('#goal-lat-DMS-sec-input-' + current).val());
       if( isNaN(dec) || isNaN(mins) || isNaN(secs)) throw "Bad input lat DMS";
      lat = dec + (mins/60 + secs/3600);
    }
    if(long_format == 0){
      //decimal-degrees mode
      long = parseFloat($('#goal-long-DD-dec-deg-input-'+current).val());
        if(isNaN(lat)) throw "Bad input lat DD";
    }
    else if(long_format == 1){
      //degrees-decimal-minutes mode
      let dec = parseFloat($('#goal-long-DDM-deg-input-'+current).val());
      let min = parseFloat($('#goal-long-DDM-dec-min-input-'+current).val());
       if(isNaN(dec) || isNaN(min)) throw "Bad input lat DDM"
      long = dec+ (min/60);

    }
    else if(long_format == 2){
      //degress-minutes-seconds mode
      let dec = parseFloat($('#goal-long-DMS-deg-input-'  + current).val());
      let mins = parseFloat($('#goal-long-DMS-min-input-' + current).val());
      let secs = parseFloat($('#goal-long-DMS-sec-input-' + current).val());
       if( isNaN(dec) || isNaN(mins) || isNaN(secs)) throw "Bad input lat DMS";
      long = dec + (mins/60 + secs/3600);
    }
    
    let latlongpair = {
      latitude : lat,
      longitude : long
    }
    $('#goal-display-lat').text(lat);
    $('#goal-display-long').text(long);

    navQueue.enqueue(latlongpair);
  }
  catch(e){
    appendToConsole(e);
  }
    console.log(navQueue);
  }
     

  function CreateGoalLatitudeHandler(current){

    $("#goal-lat-decimal-degrees-btn-" + current).on('click', function(event){
      
      event.preventDefault();
      $(this).data('clicked', true);
      $('#goal-lat-select-format-btn-' + current).dropdown('toggle');
      $('#goal-lat-select-format-btn-' + current).detach();

     
      $.ajax('/navigation/inputTemplates/goal-DD/'+'lat/'+current, {
          success: function (result) {
          $('#goal-lat-input-group-' + current).append(result);
  
       }})  



      })

     $("#goal-lat-degrees-decimal-minutes-btn-" + current).on('click', function(event){

      event.preventDefault();
      $(this).data('clicked', true);
      $('#goal-lat-select-format-btn-' + current).dropdown('toggle');
      $('#goal-lat-select-format-btn-' + current).detach();

     
      $.ajax('/navigation/inputTemplates/goal-DDM/'+'lat/'+current, {
          success: function (result) {
          $('#goal-lat-input-group-' + current).append(result);
  
       }})  


    })
     $("#goal-lat-degrees-minutes-seconds-btn-" + current).on('click', function(event){
      event.preventDefault();
      $(this).data('clicked', true);
      $('#goal-lat-select-format-btn-' + current).dropdown('toggle');
      $('#goal-lat-select-format-btn-' + current).detach();

     
      $.ajax('/navigation/inputTemplates/goal-DMS/'+'lat/'+current, {
          success: function (result) {
          $('#goal-lat-input-group-' + current).append(result);
  
       }})  
     
    })
     
  }



    function CreateGoalLongtitudeHandler(current){
    $("#goal-long-decimal-degrees-btn-" + current).on('click', function(event){
      
      event.preventDefault();
      $(this).data('clicked', true);
      $('#goal-long-select-format-btn-' + current).dropdown('toggle');
      $('#goal-long-select-format-btn-' + current).detach();

     
      $.ajax('/navigation/inputTemplates/goal-DD/'+'long/'+current, {
          success: function (result) {
          $('#goal-long-input-group-' + current).append(result);
  
       }})  



      })

    $("#goal-long-degrees-decimal-minutes-btn-" + current).on('click', function(event){
     event.preventDefault();
      $(this).data('clicked', true);
      $('#goal-long-select-format-btn-' + current).dropdown('toggle');
      $('#goal-long-select-format-btn-' + current).detach();

     
      $.ajax('/navigation/inputTemplates/goal-DDM/'+'long/'+current, {
          success: function (result) {
          $('#goal-long-input-group-' + current).append(result);
  
       }})  

 
    })
    $("#goal-long-degrees-minutes-seconds-btn-" + current).on('click', function(event){

       event.preventDefault();
      $(this).data('clicked', true);
      $('#goal-long-select-format-btn-' + current).dropdown('toggle');
      $('#goal-long-select-format-btn-' + current).detach();

     
      $.ajax('/navigation/inputTemplates/goal-DMS/'+'long/'+current, {
          success: function (result) {
          $('#goal-long-input-group-' + current).append(result);
  
       }})  
 
    })
     

    }
    function CreateGoalButtons(current){


      $.ajax('navigation/inputTemplates/goal-buttons/'+current, {
          success: function (result) {
                $("#goal-buttons-input-group-" + current).append(result);
      
      CreateConfirmButtonHandler(current);
      CreateChangeButtonHandler(current);
      CreateDeleteButtonHandler(current);   
       }})  

    }
    function CreateConfirmButtonHandler(current){
  $('#confirm-btn-' + current).on('click' , function(event){
    let lat_format = -1;
    let long_format = -1;
    if($("#goal-lat-decimal-degrees-btn-"+current).data('clicked'))
      lat_format = 0;
    else if($("#goal-lat-degrees-decimal-minutes-btn-"+current).data('clicked'))
      lat_format = 1;
    else if($("#goal-lat-degrees-minutes-seconds-btn-"+current).data('clicked'))
      lat_format = 2;
    else appendToConsole("error?");

    if($("#goal-long-decimal-degrees-btn-"+current).data('clicked'))
      long_format = 0;
    else if($("#goal-long-degrees-decimal-minutes-btn-"+current).data('clicked'))
      long_format = 1;
    else if($("#goal-long-degrees-minutes-seconds-btn-"+current).data('clicked'))
      long_format = 2;
    else appendToConsole("error?");
    
    //disable text fields
    $('#goal-input-fieldset-' + current).prop('disabled',true);

  $('#change-btn-' + current).prop('disabled', false);
  $('#confirm-btn-' + current).prop('disabled', true);

   InsertDataInQueue(current,lat_format,long_format);



})    
}
function CreateChangeButtonHandler(current) {
   $('#change-btn-' + current).on('click' , function(event){
  
  
    $('#goal-input-fieldset-' + current).prop('disabled',false);
  $('#change-btn-' + current).prop('disabled', false);
  $('#confirm-btn-' + current).prop('disabled', false);  
})
}
function CreateDeleteButtonHandler(current) {

   $('#delete-btn-' + current).on('click' , function(event){
    appendToConsole(current + ' delete was clicked')
    $('#new-goal-btn-'+current).remove();

  //gotta remove the data from the queue
  navQueue.remove(current);
  console.log(navQueue);
})  
}

})

