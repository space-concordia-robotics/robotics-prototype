// Handle and log ajax errors from flask server
function flaskError (jqXHR, exception, ajax_url) {
  full_url = 'localhost:5000' + ajax_url
  if (jqXHR.status === 0) {
    error_msg = 'cannot connect to ' + full_url
  } else if (jqXHR.status == 404) {
    error_msg = 'error [404] when accessing ' + full_url
  } else if (jqXHR.status == 500) {
    error_msg = 'error [500] when accessing ' + full_url
  } else if (exception === 'parsererror') {
    error_msg = 'failed to request JSON from ' + full_url
  } else if (exception === 'timeout') {
    error_msg = 'time out accessing ' + full_url
  } else if (exception === 'abort') {
    error_msg = 'Ajax request of ' + full_url + ' aborted'
  } else {
    error_msg = 'Uncaught Error: ' + jqXHR.responseText + ' in ' + full_url
  }
  appendToConsole(error_msg)
}
