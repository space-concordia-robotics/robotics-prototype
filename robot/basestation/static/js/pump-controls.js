$(document).ready(() => {

    let pumpDriveToggles = [
      '#pump1-drive-toggle',
      '#pump2-drive-toggle',
      '#pump3-drive-toggle',
      '#pump4-drive-toggle',
      '#pump5-drive-toggle'
    ]

    for (let i = 0; i < pumpDriveToggles.length; i++) {
      $(pumpDriveToggles[i]).on('click', function (event) {
        event.preventDefault()
        let cmd = 'p' + (i + 1)

        if (!isScienceActivated()) {
          return
        }

        // click makes it checked during this time, so trying to enable
        if ($(pumpDriveToggles[i]).is(':checked')) {
          sendRequest('Science', cmd, function (msgs) {
            console.log('msgs', msgs)
            if (msgs[1].includes(cmd + ' done')) {
              $(pumpDriveToggles[i])[0].checked = true
            } else {
              $(pumpDriveToggles[i])[0].checked = false
            }
          })
        } else {
          // stop all pumps
          sendRequest('Science', 'ps', function (msgs) {
            if (msgs[1].includes('ps done')) {
              toggleOffAllPumps()
            } else {
              appendToConsole('Something went wrong')
            }
          })
        }
      })
    }

    $('#pump-dir-toggle').click(function (event) {
      event.preventDefault()

      if (!isScienceActivated()) {
        return
      }
      // click makes it checked during this time, so trying to enable
      if ($('#pump-dir-toggle').is(':checked')) {
        sendRequest('Science', 'pd1', function (msgs) {
          if (msgs[1].includes('OUT')) {
            appendToConsole('Success')
          } else {
            appendToConsole('Something went wrong')
          }
        })
      } else {
        sendRequest('Science', 'pd0', function (msgs) {
          if (msgs[1].includes('IN')) {
            appendToConsole('Success')
          } else {
            appendToConsole('Something went wrong')
          }
        })
      }
    })
})
