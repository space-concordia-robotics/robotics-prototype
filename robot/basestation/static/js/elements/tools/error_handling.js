function flaskError (jqXHR, exception) {
  if (jqXHR.status === 0) {
    error_msg = 'cannot connect verify network'
  } else if (jqXHR.status == 404) {
    error_msg = 'requested page not found [404]'
  } else if (jqXHR.status == 500) {
    error_msg = 'internal Server Error [500]'
  } else if (exception === 'parsererror') {
    error_msg = 'requested JSON parse failed'
  } else if (exception === 'timeout') {
    error_msg = 'time out error'
  } else if (exception === 'abort') {
    error_msg = 'Ajax request aborted'
  } else {
    error_msg = 'Uncaught Error. ' + jqXHR.responseText
  }
  appendToConsole(error_msg)
}
