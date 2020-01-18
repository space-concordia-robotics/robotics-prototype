$(document).ready(() => { 
  // setup a subscriber for the rover_position topic
  let rover_position_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'rover_position',
    messageType: 'geometry_msgs/Point'
  })
  rover_position_listener.subscribe(function (message) {
    $('#rover-latitude').text(message.x)
    $('#rover-longtitude').text(message.y)
    $('#rover-heading').text(message.z)
  })
  // setup a subscriber for the antenna_goal topic
  let antenna_goal_listener = new ROSLIB.Topic({
    name: 'antenna_goal',
    messageType: 'geometry_msgs/Point',
    ros: ros
  })
  antenna_goal_listener.subscribe(function (message) {
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
  let has_gps_goal = new ROSLIB.Param({
    ros : ros,
    name : 'has_gps_goal'
  })
  let got_antenna_pos =new ROSLIB.Param({
    ros : ros,
    name : 'got_antenna_pos'
  }) 
  // setup a subscriber for the rover_goal topic
  let rover_goal_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'rover_goal',
    messageType: 'geometry_msgs/Point'
  })

  rover_goal_listener.subscribe(function (message) {
    if(parseFloat(message.y) < 50) {

      //remove the most recent coordinate pair from the GUI
      for(i = 0 ; i < count ; i++){

        if($('#new-goal-btn-'+i).length){
          $('#new-goal-btn-'+i).remove();
          break;
        }
      }
      
      console.log('next coordinate!');
      navQueue.data[0] = null;
      
      while(navQueue.top() == null){
        navQueue.dequeue();
      }
      
      goal_latitude.set(parseFloat(navQueue.top().latitude));
      goal_longitude.set(parseFloat(navQueue.top().longtitude));
      
      has_gps_goal.set(false)

      console.log(navQueue);
   //   $('#goal-display-lat').text(navQueue.top().latitude);
 }
 $('#goal-rover-heading').text(parseFloat(message.x).toFixed(3))
 $('#goal-distance').text(parseFloat(message.y).toFixed(2))
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

var navQueue = new NavigationQueue();
var count = 0;

  function initNavigationPanel () {

    let hasAntennaParams = true
    got_antenna_pos.set(false)    

    antenna_latitude.get(function(lat){
      if(lat != null){
        antenna_longitude.get(function(long){
          if(long != null){
            antenna_start_dir.get(function(dir){
              if(dir != null){
                appendToConsole('Antenna parameters already set')

                
                $.get({
                  url : '/navigation/cached_content/antenna_stats',
                  success: function(result){
                        $('#antenna-stats-list').append(result);
                  },

                })
                $.get({
                  url : '/navigation/cached_content/antenna_modal',
                  success: function(result){
                        
                       
                       let data =  $('#antenna-select-lat-format').detach();
                       let data2 =  $('#antenna-select-long-format').detach();
                       
                      $('#antenna-modal-body').empty();
                      $('#antenna-modal-body').append(result);
                       console.log($('#antenna-lat-fieldset').data('format'))
                      

                        CreateAntennaLatitudeChangeButtonHandler(data)
                        CreateAntennaLongtitudeChangeButtonHandler(data2)
                        CreateAntennaBearingChangeButtonHandler()
                  }
                  
                })
              
          } 
          else{
            appendToConsole('Set antenna parameters!')
          }
        })

          }
          else{
            appendToConsole('Set antenna parameters!')
          } 
        })
      }
      else{
        appendToConsole('Set antenna parameters!')
      }
    })

    goal_latitude.get(function(lat){
      if(lat != null){
        goal_longitude.get(function(long){
          if(long != null){

                  $.get({
                  url : '/navigation/cached_content/goal_modal',
                  success:function(result){
                    console.log(result)

                    $('#goal-modal-body').empty();
                    $('#goal-modal-body').append(result);

                    count = $('#goal-modal-body-content').attr('count');
                    $('#goal-stats-lat').text(lat)
                    $('#goal-stats-long').text(long)

                    for( i = 0 ; i < count ; i++){
                      if($('#new-goal-btn-'+i).length){
                        CreateGoalChangeButtonHandler(i);
                        CreateGoalDeleteButtonHandler(i);
                        CreateGoalConfirmButtonHandler(i);
                      }
                    }

                  }
                })
                  $.get({
                    url : '/navigation/cached_content/navQueue',
                    success:function(result){
                      
                      navQueue.data = JSON.parse(result).data
                      navQueue.shift = JSON.parse(result).shift
                      console.log(navQueue)
                    }
                  })
          }
          else{
            console.log("Enter goal longtitude")
          }
        })
      }
      else{
        console.log("Enter goal latitude!")
      }
    })
  }
  //theres probably a better place to put this, this is just a temporary fix
  initNavigationPanel();
  
// 0 1 2 3 4 5
function NavigationQueue() {
  this.data = [];
  this.shift = 0;
  flag = true;  

  this.enqueue = function(item){
    this.data.push(item);
    if(flag){
      has_gps_goal.set(false)
      goal_latitude.set(item.latitude);
      goal_longitude.set(item.longtitude);
      flag = false;
    }
  }
  this.dequeue = function(){
    this.data.shift();
    this.shift++;
  }
  this.top = function(){
    return this.data[0];
  }
  this.kill = function(index){
   this.data[index-this.shift] = null;

   //if the current top is deleted, gotta adjust the goal_lat and goal_long parmamters
   if(index - this.shift == 0) {

     while(navQueue.top() == null){
      navQueue.dequeue();
    }

    goal_latitude.set(this.top().latitude)
    goal_longitude.set(this.top().longtitude)
  }
}

this.change = function(index,lat,long){
  console.log('index of change ' + index);
  this.data[index-this.shift].latitude = lat;
  this.data[index-this.shift].longtitude = long;

  if( (index-this.shift) ==0) {
    goal_latitude.set(lat)
    goal_longitude.set(long)
    has_gps_goal.set(false)
  }
} 
}




$('#antenna-select-lat-format-btn').on('click',function(event){
  event.preventDefault();
  CreateAntennaLatitudeHandler();

})                
$('#antenna-select-long-format-btn').on('click',function(event){
  event.preventDefault();
  CreateAntennaLongtitudeHandler();


})             
            

$('#antenna-confirm-btn').on('click', function(event){

    event.preventDefault();
    //check if the inputs are valid
    let lat = -1;
    let long = -1;

    

    try{
      if($("#antenna-lat-fieldset").attr('format') == 'DD') {
       // $("#antenna-lat-decimal-degrees-btn").data('clicked', false)
       let decdeg = parseFloat( ($("#antenna-lat-DD-dec-deg-input").val()) ) ;
       $("#antenna-lat-DD-dec-deg-input").attr("value" , decdeg);
     
       if(isNaN(decdeg)) throw "Bad Input lat DD";
       lat  = decdeg;
      }
     else if($("#antenna-lat-fieldset").attr('format') == 'DDM'){
     // $("#antenna-lat-degrees-decimal-minutes-btn").data('clicked',false) 
      let dec = parseFloat($('#antenna-lat-DDM-deg-input').val());
      let min = parseFloat($('#antenna-lat-DDM-dec-min-input').val());
      if( isNaN(dec) || isNaN (min)) throw "Bad input lat DDM";

      $("#antenna-lat-DDM-deg-input").attr("value", dec)
      $("#antenna-lat-DDM-dec-min-input").attr("value", min)
      
      lat = dec+ (min/60);

    } 
    else if($("#antenna-lat-fieldset").attr('format') == 'DMS'){
     // $("#antenna-lat-degrees-minutes-seconds-btn").data('clicked',false)

      let dec = parseFloat($('#antenna-lat-DMS-deg-input').val());
      let mins = parseFloat($('#antenna-lat-DMS-min-input').val());
      let secs = parseFloat($('#antenna-lat-DMS-sec-input').val());
      if( isNaN(dec) || isNaN(mins) || isNaN(secs)) throw "Bad input lat DMS";

      $("#antenna-lat-DMS-deg-input").attr("value",dec);
      $("#antenna-lat-DMS-min-input").attr("value",mins);
      $("#antenna-lat-DMS-sec-input").attr("value",secs);
      
      lat = dec + (mins/60 + secs/3600);

    }  

    if($("#antenna-long-fieldset").attr('format') == 'DD') {
     // $("#antenna-long-decimal-degrees-btn").data('clicked',false) 
      let decdeg = parseFloat( ($("#antenna-long-DD-dec-deg-input").val()) ) ;
      if(isNaN(decdeg)) throw "Bad Input long DD";
      
       $("#antenna-long-DD-dec-deg-input").attr("value" , decdeg);
       long  = decdeg;
    }
    else if($("#antenna-long-fieldset").attr('format') == 'DDM' ){
    //  $("#antenna-long-degrees-decimal-minutes-btn").data('clicked',false)
      let dec = parseFloat($('#antenna-long-DDM-deg-input').val());
      let min = parseFloat($('#antenna-long-DDM-dec-min-input').val());
      if( isNaN(dec) || isNaN (min)) throw "Bad input long DDM";

      $("#antenna-long-DDM-deg-input").attr("value", dec)
      $("#antenna-long-DDM-dec-min-input").attr("value", min)
    
      long = dec+ (min/60);
    } 
    else if($("#antenna-long-fieldset").attr('format') == 'DMS'){
    //  $("#antenna-long-degrees-minutes-seconds-btn").data('clicked',false)
      let dec = parseFloat($('#antenna-long-DMS-deg-input').val());
      let mins = parseFloat($('#antenna-long-DMS-min-input').val());
      let secs = parseFloat($('#antenna-long-DMS-sec-input').val());
      if( isNaN(dec) || isNaN(mins) || isNaN(secs)) throw "Bad input long DMS";

      $("#antenna-long-DMS-deg-input").attr("value",dec);
      $("#antenna-long-DMS-min-input").attr("value",mins);
      $("#antenna-long-DMS-sec-input").attr("value",secs);
      
      long = dec + (mins/60 + secs/3600);
    }  
    let bearing = parseFloat($("#antenna-bearing-input").val());
    if(isNaN(bearing)) throw "Bad input bearing";
    $("#antenna-bearing-input").attr("value",bearing)
          //ROS params
          antenna_latitude.set(lat);
          antenna_longitude.set(long);
          antenna_start_dir.set(bearing);

          appendToConsole('antenna parmameters have been set!')

          $.ajax('/navigation/inputTemplates/antenna-stats', {
            success: function (result) {

              $('#antenna-stats-list').empty();
              $('#antenna-stats-list').append(result);

            $('.antenna-input-field').prop('disabled',true)
            $('.antenna-change-btn').prop('disabled',false);

            $('#antenna-confirm-btn').prop('disabled', true);
            CreateAntennaBearingChangeButtonHandler();
              

            got_antenna_pos.set(false)
              


          //update antenna diplayed fields
          
          $('#antenna-display-lat').text(lat);
          $('#antenna-display-long').text(long);
          $('#antenna-display-heading').text(bearing);

          $.post({
              url: "/navigation/cached_content/antenna_stats",
              data: $('#antenna-stats-list').html(),
             
            });
          
          $.post({
              url: "/navigation/cached_content/antenna_modal",
              data: $('#antenna-modal-body').html(),

            });




        }, error: function(result){
          console.log('error : ' + result)
        }})  

        }
        catch(e){
          appendToConsole(e);
        }


      })

function CreateAntennaLatitudeChangeButtonHandler(detachedData){

  $("#antenna-lat-DD-change-btn").on('click', function(event){
    event.preventDefault()
    
    $("#antenna-lat-DD-change-btn").prop('disabled',true)
    $("#antenna-lat-DD-dec-deg-input").prop('disabled',false)
    

    $("#antenna-lat-input-group").detach()
    $("#antenna-lat-fieldset").append(detachedData)

    $("#antenna-confirm-btn").prop('disabled',false)

    //CreateAntennaLatitudeHandler(); 
    })

      $("#antenna-lat-DDM-change-btn").on('click', function(event){
    event.preventDefault()
    
    $("#antenna-lat-DDM-change-btn").prop('disabled',true)
    $("#antenna-lat-DDM-dec-deg-input").prop('disabled',false)
    

    $("#antenna-lat-input-group").detach()
    $("#antenna-lat-fieldset").append(detachedData)

    $("#antenna-confirm-btn").prop('disabled',false)

    })
      $("#antenna-lat-DMS-change-btn").on('click', function(event){
    event.preventDefault()
    
    
    $("#antenna-lat-DMS-change-btn").prop('disabled',true)
    $("#antenna-lat-DMS-dec-deg-input").prop('disabled',false)
    

    $("#antenna-lat-input-group").detach()
    $("#antenna-lat-fieldset").append(detachedData)

    $("#antenna-confirm-btn").prop('disabled',false)

    })
  
}
function CreateAntennaLongtitudeChangeButtonHandler(detachedData){

  $("#antenna-long-DD-change-btn").on('click', function(event){
    event.preventDefault()
     
    $("#antenna-long-DD-change-btn").prop('disabled',true)
    $("#antenna-long-DD-dec-deg-input").prop('disabled',false)
    
    $("#antenna-long-input-group").detach()
    $("#antenna-long-fieldset").append(detachedData)

    $("#antenna-confirm-btn").prop('disabled',false)
    })

    $("#antenna-long-DDM-change-btn").on('click', function(event){
    event.preventDefault()
     
    $("#antenna-long-DDM-change-btn").prop('disabled',true)
    $("#antenna-long-DDM-dec-deg-input").prop('disabled',false)
    
    $("#antenna-long-input-group").detach()
    $("#antenna-long-fieldset").append(detachedData)

    $("#antenna-confirm-btn").prop('disabled',false)
    })

    $("#antenna-long-DMS-change-btn").on('click', function(event){
    event.preventDefault()
     
    $("#antenna-long-DMS-change-btn").prop('disabled',true)
    $("#antenna-long-DMS-dec-deg-input").prop('disabled',false)
    
    $("#antenna-long-input-group").detach()
    $("#antenna-long-fieldset").append(detachedData)

    $("#antenna-confirm-btn").prop('disabled',false)
    })
}
function CreateAntennaBearingChangeButtonHandler(){
  $("#antenna-bearing-change-btn").on('click' , function(event){
    
    $("#antenna-bearing-input").prop('disabled',false)
    $("#antenna-bearing-change-btn").prop('disabled',false)
    $("#antenna-confirm-btn").prop('disabled',false)
  })
}

function CreateAntennaLatitudeHandler(){
  
  $('#antenna-lat-decimal-degrees-btn').unbind('click').on('click', function (event) {
      
    $("#antenna-lat-fieldset").attr('format', 'DD');
    
    $("#antenna-select-lat-format-btn").dropdown('toggle');
    detachedData = $('#antenna-select-lat-format').detach();

    console.log($("#antenna-lat-fieldset").data());

    

    $.ajax('/navigation/inputTemplates/antenna-DD/'+'lat', {
      success: function (result) {
        $("#antenna-lat-fieldset").append(result);
            CreateAntennaLatitudeChangeButtonHandler(detachedData);

      }})  
  })
  
  $('#antenna-lat-degrees-decimal-minutes-btn').unbind('click').on('click', function (event) {
    event.preventDefault();
    
    $("#antenna-lat-fieldset").attr('format', 'DDM');
    
    $("#antenna-select-lat-format-btn").dropdown('toggle');
    detachedData = $('#antenna-select-lat-format').detach();


    $.ajax('/navigation/inputTemplates/antenna-DDM/'+'lat', {
      success: function (result) {
        $("#antenna-lat-fieldset").append(result);

    CreateAntennaLatitudeChangeButtonHandler(detachedData);
      }})  



  })
  $('#antenna-lat-degrees-minutes-seconds-btn').unbind().on('click', function (event) {
   event.preventDefault();

   $("#antenna-lat-fieldset").attr('format','DMS');
    

    $("#antenna-select-lat-format-btn").dropdown('toggle');
    detachedData = $('#antenna-select-lat-format').detach();


   $.ajax('/navigation/inputTemplates/antenna-DMS/'+'lat', {
    success: function (result) {
      $("#antenna-lat-fieldset").append(result);

   CreateAntennaLatitudeChangeButtonHandler(detachedData);
    }})  
 })
}

function CreateAntennaLongtitudeHandler(){
 $('#antenna-long-decimal-degrees-btn').unbind().on('click', function (event) {
  event.preventDefault();
    
    $("#antenna-long-fieldset").attr('format','DD');
    
    $("#antenna-select-long-format-btn").dropdown('toggle');
    detachedData = $('#antenna-select-long-format').detach();

  $.ajax('/navigation/inputTemplates/antenna-DD/'+'long', {
    success: function (result) {
      $("#antenna-long-fieldset").append(result);
      CreateAntennaLongtitudeChangeButtonHandler(detachedData)
    }})  


})

 $('#antenna-long-degrees-decimal-minutes-btn').unbind().on('click', function (event) {

   event.preventDefault();

   $("#antenna-long-fieldset").attr('format','DDM');
    
    $("#antenna-select-long-format-btn").dropdown('toggle');
    detachedData = $('#antenna-select-long-format').detach();


   $.ajax('/navigation/inputTemplates/antenna-DDM/'+'long', {
    success: function (result) {
      
      $("#antenna-long-fieldset").append(result);
      CreateAntennaLongtitudeChangeButtonHandler(detachedData)

    }})  
 })
 $('#antenna-long-degrees-minutes-seconds-btn').unbind().on('click', function (event) {

  event.preventDefault();

  $("#antenna-long-fieldset").attr('format','DMS');

    $("#antenna-select-long-format-btn").dropdown('toggle');
    detachedData = $('#antenna-select-long-format').detach();


  $.ajax('/navigation/inputTemplates/antenna-DMS/'+'long', {
    success: function (result) {
      
      $("#antenna-long-fieldset").append(result);
      CreateAntennaLongtitudeChangeButtonHandler(detachedData)
  
    }})  
})
} 


$("#new-goal-coordinates-btn").on('click', function(event) {

  $.ajax('/navigation/inputTemplates/new-goal-coordinates-btn/'+count, {
    success: function (result) {
      $("#goal-modal-body-content").append(result);

      CreateGoalLatitudeHandler(count);
      CreateGoalLongtitudeHandler(count);
      CreateGoalButtons(count);

        count++;    
      }})
})

function InsertDataInQueue(current,lat_format,long_format){

  let lat = -1;
  let long = -1;
  try{
    if(lat_format == 0){
      //decimal-degrees mode
      decdeg = parseFloat($('#goal-lat-DD-dec-deg-input-'+current).val());
      if(isNaN(decdeg)) throw "Bad input lat DD";
      $('#goal-lat-DD-dec-deg-input-'+current).attr("value",decdeg);
      lat = decdeg
    }
    else if(lat_format == 1){
      //degrees-decimal-minutes mode
      let dec = parseFloat($('#goal-lat-DDM-deg-input-'+current).val());
      let min = parseFloat($('#goal-lat-DDM-dec-min-input-'+current).val());
      if(isNaN(dec) || isNaN(min)) throw "Bad input lat DDM"
        lat = dec+ (min/60);
     $('#goal-lat-DDM-deg-input-'+current).attr("value",dec)
     $('#goal-lat-DDM-dec-min-input-'+current).attr("value",min)
       
    }
    else if(lat_format == 2){
      //degress-minutes-seconds mode
      let dec = parseFloat($('#goal-lat-DMS-deg-input-'  + current).val());
      let mins = parseFloat($('#goal-lat-DMS-min-input-' + current).val());
      let secs = parseFloat($('#goal-lat-DMS-sec-input-' + current).val());
      if( isNaN(dec) || isNaN(mins) || isNaN(secs)) throw "Bad input lat DMS";
      lat = dec + (mins/60 + secs/3600);
      $('#goal-lat-DMS-deg-input-'+current).attr("value",dec);
      $('#goal-lat-DMS-min-input-'+current).attr("value",mins);
      $('#goal-lat-DMS-sec-input-'+current).attr("value",secs);
       
    }
    if(long_format == 0){
      //decimal-degrees mode
      long = parseFloat($('#goal-long-DD-dec-deg-input-'+current).val());
      if(isNaN(lat)) throw "Bad input long DD";

         $('#goal-long-DD-dec-deg-input-'+current).attr("value",long);
    }
    else if(long_format == 1){
      //degrees-decimal-minutes mode
      let dec = parseFloat($('#goal-long-DDM-deg-input-'+current).val());
      let min = parseFloat($('#goal-long-DDM-dec-min-input-'+current).val());
      if(isNaN(dec) || isNaN(min)) throw "Bad input long DDM"
        long = dec+ (min/60);
     $('#goal-long-DDM-deg-input-'+current).attr("value",dec)
     $('#goal-long-DDM-dec-min-input-'+current).attr("value",min)
       
    } 
    else if(long_format == 2){
      //degress-minutes-seconds mode
      let dec = parseFloat($('#goal-long-DMS-deg-input-'  + current).val());
      let mins = parseFloat($('#goal-long-DMS-min-input-' + current).val());
      let secs = parseFloat($('#goal-long-DMS-sec-input-' + current).val());
      if( isNaN(dec) || isNaN(mins) || isNaN(secs)) throw "Bad input lat DMS";
      long = dec + (mins/60 + secs/3600);
      $('#goal-long-DMS-deg-input-'+current).attr("value",dec);
      $('#goal-long-DMS-min-input-'+current).attr("value",mins);
      $('#goal-long-DMS-sec-input-'+current).attr("value",secs);
       
    }
    
    $('#goal-lat-fieldset-' + current).prop('disabled',true);
    $('#goal-long-fieldset-' + current).prop('disabled',true);
  
    let latlongpair = {
      latitude : lat,
      longtitude : long
    }
          //$('#goal-stats-list').show();
          $('#goal-stats-lat').text(lat);
          $('#goal-stats-long').text(long);

          $("#goal-modal-body-content").attr("count",count)     
                
          $.post({
              url: "/navigation/cached_content/goal_modal",
              data: $('#goal-modal-body').html()          
            })

          

    if($("#change-btn-"+current).data('clicked')){
      appendToConsole('changed!')
      $(this).data('clicked', false);
      navQueue.change(current,lat,long)      
    }
    else {
      navQueue.enqueue(latlongpair);
      $.post({
            url: "/navigation/cached_content/navQueue",
            contentType: "application/json",
            data : JSON.stringify(navQueue)
          })   
    }
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
      
      CreateGoalConfirmButtonHandler(current);
      CreateGoalChangeButtonHandler(current);
      CreateGoalDeleteButtonHandler(current);   
    }})  

}
function CreateGoalConfirmButtonHandler(current){
  $('#confirm-btn-' + current).on('click' , function(event){
    let lat_format = -1;
    let long_format = -1;
    if($("#goal-lat-decimal-degrees-btn-"+current).data('clicked'))
      lat_format = 0;
    else if($("#goal-lat-degrees-decimal-minutes-btn-"+current).data('clicked'))
      lat_format = 1;
    else if($("#goal-lat-degrees-minutes-seconds-btn-"+current).data('clicked'))
      lat_format = 2;
    else appendToConsole("error with lat format");

    if($("#goal-long-decimal-degrees-btn-"+current).data('clicked'))
      long_format = 0;
    else if($("#goal-long-degrees-decimal-minutes-btn-"+current).data('clicked'))
      long_format = 1;
    else if($("#goal-long-degrees-minutes-seconds-btn-"+current).data('clicked'))
      long_format = 2;
    else appendToConsole("error with long format");
    
    //disable text fields
    $('#change-btn-' + current).prop('disabled', false);
    $('#confirm-btn-' + current).prop('disabled', true);

    $('#goal-lat-select-format-'+current).detach()
    $('#goal-long-select-format-'+current).detach()
    
    
    InsertDataInQueue(current,lat_format,long_format);



  })    
}

// 0 1 2 3 4 5 6

function CreateGoalChangeButtonHandler(current) {
 $('#change-btn-' + current).on('click' , function(event){

  $(this).data('clicked', true);
  
  $('#goal-lat-fieldset-' + current).prop('disabled',false);
  $('#goal-long-fieldset-' + current).prop('disabled',false);
  
  $('#change-btn-' + current).prop('disabled', true);
  $('#confirm-btn-' + current).prop('disabled', false);  
})
}
function CreateGoalDeleteButtonHandler(current) {

 $('#delete-btn-' + current).on('click' , function(event){
  appendToConsole(current + ' delete was clicked')
  $('#new-goal-btn-'+current).remove();

  //gotta remove the data from the queue
  navQueue.kill(current);
  console.log(navQueue);
})  
}

})

