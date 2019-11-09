$(document).ready(() => {
  // @TODO: these buttons are flipped because of wiring issues
  // fix the wiring, change back the correct values
  $('#elevator-up-btn').on('click', function (event) {
    if (!isScienceActivated()) {
      return
    }
    sendRequest('Science', 'edown', function (msgs) {
      console.log('msgs', msgs)
    })
  })

  $('#elevator-down-btn').on('click', function (event) {
    if (!isScienceActivated()) {
      return
    }
    sendRequest('Science', 'eup', function (msgs) {
      console.log('msgs', msgs)
    })
  })

  $('#elevator-max-speed-go-btn').click(function (msgs) {
    if (!isScienceActivated()) {
      return
    }

    sendRequest('Science', 'ego', function (msgs) {
      console.log('msgs', msgs)

      if (msgs[1].includes('ego done')) {
        appendToConsole('Success')
        lightUp('#elevator-max-speed-go-btn')
        greyOut('#elevator-stop-btn')
      } else {
        appendToConsole('Something went wrong')
      }
    })
  })

  $('#set-feed-go-btn').click(function (msgs) {
    if (!isScienceActivated()) {
      return
    }

    let elevatorFeed = $('#elevator-feed').val()
    let cmd = 'ef ' + elevatorFeed
    let requestedFeed = Number(elevatorFeed)

    // invalid range check
    // values under 10% don't actually move the elevator
    if (requestedFeed < 10 || requestedFeed > 100) {
      color('#elevator-feed', 'orange')
      appendToConsole('Valid ranges for elevator feed: [10, 100]')
      return
    }

    color('#elevator-feed', 'white')

    sendRequest('Science', cmd, function (msgs) {
      console.log('msgs', msgs)

      if (msgs[1].includes('elevatorfeed done')) {
        appendToConsole('Success')
        lightUp('#set-feed-go-btn')
        greyOut('#elevator-stop-btn')
      } else {
        appendToConsole('Something went wrong')
      }
    })
  })

  // elevator - set distance command
  $('#set-distance-go-btn').click(function (msgs) {
    if (!isScienceActivated()) {
      return
    }

    let elevatorDistance = $('#elevator-distance').val()
    let cmd = 'elevatordistance ' + elevatorDistance
    let requestedDistance = Number(elevatorDistance)

    // invalid range check
    // if the system on the MCU were closed loop we would do it based off current position
    // otherwise we assume there will be limit switches for protection
    if (requestedDistance <= 0 || requestedDistance > 15) {
      color('#elevator-distance', 'orange')
      appendToConsole('Valid ranges for elevator feed: (0, 15]')
      return
    }

    color('#elevator-distance', 'white')

    // check if feed set
    let feed = $('#elevator-feed').val()

    if (feed != '') {
      feed = Number(feed)
      // anything under 20% is too slow and takes _very_ long time to reach 1 inch distance
      if (isNumeric(feed) && (feed >= 20 && feed <= 100)) {
        cmd += ' ' + feed
      } else {
        color('#elevator-feed', 'orange')
        appendToConsole('Illegal parameter set for feed')
        appendToConsole('Legal range of integers: [10, 100]')
        return
      }
    }

    // if we made it this far then reset the input field to normal background
    color('#elevator-feed', 'white')

    sendRequest('Science', cmd, function (msgs) {
      console.log('msgs', msgs)

      if (msgs[1].includes('elevatordistance done')) {
        appendToConsole('Success')
        lightUp('#set-distance-go-btn')
        greyOut('#elevator-stop-btn')
      } else {
        appendToConsole('Something went wrong')
      }
    })
  })

  $('#elevator-stop-btn').click(function (msgs) {
    if (!isScienceActivated()) {
      return
    }

    sendRequest('Science', 'es', function (msgs) {
      console.log('msgs', msgs)

      if (msgs[1].includes('es done')) {
        appendToConsole('Success')
        lightUp('#elevator-stop-btn')
        greyOut('#elevator-max-speed-go-btn')
        greyOut('#set-feed-go-btn')
        greyOut('#set-distance-go-btn')
      } else {
        appendToConsole('Something went wrong')
      }
    })
  })
})
