$(document).ready(() => {
    $('#drill-max-speed-go-btn').click(function (msgs) {
      if (!isScienceActivated()) {
        return
      }

      sendRequest('Science', 'dgo', function (msgs) {
        console.log('msgs', msgs)

        if (msgs[1].includes('dgo done')) {
          appendToConsole('Success')
          lightUp('#drill-max-speed-go-btn')
          greyOut('#drill-stop-btn')
        } else {
          appendToConsole('Something went wrong')
        }
      })
    })

    $('#drill-ccw-btn').on('click', function (event) {
      if (!isScienceActivated()) {
        return
      }
      sendRequest('Science', 'dccw', function (msgs) {
        console.log('msgs', msgs)
      })
    })

    $('#drill-cw-btn').on('click', function (event) {
      if (!isScienceActivated()) {
        return
      }
      sendRequest('Science', 'dcw', function (msgs) {
        console.log('msgs', msgs)
      })
    })

    $('#set-speed-go-btn').click(function (msgs) {
      if (!isScienceActivated()) {
        return
      }

      let drillSpeed = $('#drill-speed').val()
      let cmd = 'drillspeed ' + drillSpeed
      let requestedSpeed = Number(drillSpeed)

      // invalid range check
      // values under 6% don't actually rotate the drill
      if (requestedSpeed < 6 || requestedSpeed > 100) {
        color('#drill-speed', 'orange')
        appendToConsole('Valid ranges for drill speed: [7, 100]')
        return
      }

      color('#drill-speed', 'white')

      sendRequest('Science', cmd, function (msgs) {
        console.log('msgs', msgs)

        if (msgs[1].includes('drillspeed done')) {
          appendToConsole('Success')
          lightUp('#set-speed-go-btn')
          greyOut('#drill-stop-btn')
        } else {
          appendToConsole('Something went wrong')
        }
      })
    })

    $('#set-time-go-btn').click(function (msgs) {
      if (!isScienceActivated()) {
        return
      }

      let drillTime = $('#drill-time').val()
      let cmd = 'drilltime ' + drillTime
      let requestedTime = Number(drillTime)

      // ensure time at least one second
      if (requestedTime < 1) {
        color('#drill-time', 'orange')
        appendToConsole('Valid ranges for drill speed: [1, ∞)')
        return
      }

      color('#drill-time', 'white')

      // check if drillspeed set
      let drillSpeed = $('#drill-speed').val()

      if (isNumeric(drillSpeed)) {
        cmd += ' ' + drillSpeed
      }

      sendRequest('Science', cmd, function (msgs) {
        console.log('msgs', msgs)

        if (msgs[1].includes('drilltime done')) {
          appendToConsole('Success')
          lightUp('#set-time-go-btn')
          greyOut('#drill-stop-btn')
        } else {
          appendToConsole('Something went wrong')
        }
      })
    })

    $('#drill-stop-btn').click(function (msgs) {
      if (!isScienceActivated()) {
        return
      }

      // drill stop
      sendRequest('Science', 'ds', function (msgs) {
        console.log('msgs', msgs)

        if (msgs[1].includes('ds done')) {
          appendToConsole('Success')
          lightUp('#drill-stop-btn')
          greyOut('#drill-max-speed-go-btn')
          greyOut('#set-speed-go-btn')
          greyOut('#set-time-go-btn')
        } else {
          appendToConsole('Something went wrong')
        }
      })
    })
})
