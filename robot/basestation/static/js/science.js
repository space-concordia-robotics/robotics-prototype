// turn table graphics
const twopi = 2 * Math.PI
const pi = Math.PI

run = async () => {
  const canvas = document.getElementById('myCanvas')
  const w = canvas.width
  const h = canvas.height
  const context = canvas.getContext('2d')

  const x = 0.8 // at x=1 circle touches canvas edge
  const r = x * 0.5 * Math.min(w, h)

  // BEGIN: GET WHEEL PARAMS FROM SERVER
  const response1 = await fetch('http://localhost:5000/numSections')
  const n = Number(await response1.text())

  const response2 = await fetch('http://localhost:5000/initialSection')
  const initial_section = Number(await response2.text())
  // END: GET WHEEL PARAMS FROM SERVER

  let i = (n - initial_section) % n

  context.fillStyle = '#FFFFFF'
  context.fillRect(0, 0, w, h)

  drawWheel(context, w, h, r, n, ((-i + n) % n) * (twopi / n))

  const nframes = 100
  const d = twopi / n / nframes

  const button_rpos = document.getElementById('button_rpos')
  window.rotatePos = async () => {
    button_rpos.disabled = true
    button_rneg.disabled = true

    // wait for server to finish rotating
    await fetch('http://localhost:5000/rotatePos')

    const start = (((-i + n) % n) * twopi) / n
    for (let [a, z] = [start, 0]; z < nframes; [a, z] = [a - d, z + 1]) {
      await timer(1)
      drawWheel(context, w, h, r, n, a)
    }

    i = (i + 1) % n
    drawWheel(context, w, h, r, n, (-i * twopi) / n)

    button_rpos.disabled = false
    button_rneg.disabled = false
  }
  button_rpos.addEventListener('click', window.rotatePos)

  const button_rneg = document.getElementById('button_rneg')
  window.rotateNeg = async () => {
    button_rpos.disabled = true
    button_rneg.disabled = true

    // wait for server to finish rotating
    await fetch('http://localhost:5000/rotateNeg')

    const start = (-i * twopi) / n
    for (let [a, z] = [start, 0]; z < nframes; [a, z] = [a + d, z + 1]) {
      await timer(1)
      drawWheel(context, w, h, r, n, a)
    }

    i = (i + n - 1) % n
    drawWheel(context, w, h, r, n, (-i * twopi) / n)

    button_rpos.disabled = false
    button_rneg.disabled = false
  }
  button_rneg.addEventListener('click', rotateNeg)
}

/**
 * ctx: context
 * w, h: width and height of canvas
 * r: radius of wheel
 * n: number of sections of wheel
 * a: current angle, 0 <= a <= twopi
 */
const drawWheel = (ctx, w, h, r, n, a) => {
  const mx = w / 2
  const my = h / 2

  // setup
  const oldFillStyle = ctx.fillStyle
  const oldLineWidth = ctx.lineWidth
  const oldFont = ctx.font
  const oldTextAlign = ctx.textAlign

  // fill circle
  ctx.fillStyle = '#DBDBDB'
  ctx.beginPath()
  ctx.arc(mx, my, r, 0, twopi)
  ctx.closePath()
  ctx.fill()

  // outline circle
  ctx.fillStyle = 'black'
  ctx.lineWidth = '3.0'
  ctx.beginPath()
  ctx.arc(mx, my, r, 0, twopi)
  ctx.closePath()
  ctx.stroke()

  // draw special background on section n-1
  ctx.fillStyle = '#F5F5F5'
  ctx.lineWidth = '1.0'
  ctx.beginPath()
  ctx.moveTo(mx + Math.cos(a) * r * 0.7, my + Math.sin(a) * r * 0.7)
  ctx.lineTo(mx + Math.cos(a) * r * 0.95, my + Math.sin(a) * r * 0.95)
  ctx.lineTo(
    mx + Math.cos(a + twopi / n) * r * 0.95,
    my + Math.sin(a + twopi / n) * r * 0.95
  )
  ctx.lineTo(
    mx + Math.cos(a + twopi / n) * r * 0.7,
    my + Math.sin(a + twopi / n) * r * 0.7
  )
  ctx.closePath()
  ctx.fill()
  ctx.stroke()

  // draw sections
  ctx.fillStyle = 'black'
  ctx.lineWidth = '2.0'
  ctx.beginPath()
  for (let z = 0; z < n; z++) {
    let a_ = a - z * (twopi / n)
    let x = Math.cos(a_) * r
    let y = Math.sin(a_) * r
    ctx.moveTo(mx, my)
    ctx.lineTo(mx + x, my + y)
  }
  ctx.stroke()

  // draw numbers
  let oldA = a
  a -= twopi / (n * 2) // add half a section to a
  ctx.font = '18px serif'
  ctx.fillStyle = 'black'
  ctx.textAlign = 'center'
  for (let z = 0; z < n; z++) {
    let a_ = a - z * (twopi / n)
    let x = Math.cos(a_) * r * 0.8
    let y = Math.sin(a_) * r * 0.8
    ctx.fillText(`${z}`, mx + x, my + y)
  }
  a = oldA

  // draw wheel pointer
  // circle
  ctx.fillStyle = '#525252'
  ctx.lineWidth = '3.0'
  ctx.beginPath()
  ctx.arc(mx, my, r * 0.3, 0, twopi)
  ctx.closePath()
  ctx.fill()
  ctx.stroke()
  // pointer
  ctx.beginPath()
  ctx.moveTo(
    mx + Math.cos(-twopi / n) * r * 0.3,
    my + Math.sin(-twopi / n) * r * 0.3
  )
  ctx.lineTo(mx + Math.cos(-pi / n) * r * 0.5, my + Math.sin(-pi / n) * r * 0.5)
  ctx.lineTo(mx + Math.cos(0) * r * 0.3, my + Math.sin(0) * r * 0.3)
  ctx.closePath()
  ctx.fill()
  ctx.beginPath()
  ctx.moveTo(
    mx + Math.cos(-twopi / n) * r * 0.3,
    my + Math.sin(-twopi / n) * r * 0.3
  )
  ctx.lineTo(mx + Math.cos(-pi / n) * r * 0.5, my + Math.sin(-pi / n) * r * 0.5)
  ctx.lineTo(mx + Math.cos(0) * r * 0.3, my + Math.sin(0) * r * 0.3)
  ctx.stroke()
  // text
  ctx.font = '30px arial'
  ctx.fillStyle = 'white'
  ctx.fillText('current', mx, my - 8)
  ctx.fillText('vial', mx, my + 22)

  // teardown
  ctx.fillStyle = oldFillStyle
  ctx.lineWidth = oldLineWidth
  ctx.font = oldFont
  ctx.textAlign = oldTextAlign
}

const timer = delay => {
  return new Promise(function (resolve) {
    setTimeout(resolve, delay)
  })
}

$(document).ready(function () {
  // ROS related stuff
  $('#science-listener-btn').on('click', function (event) {
    event.preventDefault()
    // click makes it checked during this time, so trying to enable
    if ($('#science-listener-btn').is(':checked')) {
      let serialType = 'uart'
      if (
        $('#serial-type')
          .text()
          .includes('Serial')
      ) {
        appendToConsole('Select a serial type!')
      } else if (
        $('button#mux')
          .text()
          .includes('Science')
      ) {
        serialType = $('#serial-type')
          .text()
          .trim()
        requestTask(
          'science_listener',
          1,
          '#science-listener-btn',
          function (msgs) {
            console.log(msgs)
            if (msgs.length == 2) {
              if (msgs[1].includes('already running') || msgs[0] == true) {
                $('#science-listener-btn')[0].checked = true
              } else {
                $('#science-listener-btn')[0].checked = false
              }
            }
          },
          serialType
        )
      } else {
        appendToConsole(
          'Cannot turn science listener on if not in science mux channel!'
        )
      }
    } else {
      if ($('#activate-science-btn').is(':checked')) {
        appendToConsole('Deactivate the science MCU before closing listener!')
        return
      }
      // closing arm listener
      requestTask('science_listener', 0, '#science-listener-btn', function (
        msgs
      ) {
        console.log('msgs[0]', msgs[0])
        if (msgs.length == 2) {
          console.log('msgs[1]', msgs[1])
          if (msgs[1].includes('already running')) {
            $('#science-listener-btn')[0].checked = true
          } else {
            $('#science-listener-btn')[0].checked = false
          }
        } else {
          if (msgs[0]) {
            $('#science-listener-btn')[0].checked = true
          } else {
            $('#science-listener-btn')[0].checked = false
          }
        }
      })
    }
  })

  $('#activate-science-btn').on('click', function (event) {
    event.preventDefault()
    // click makes it checked during this time, so trying to enable
    if (!$('#science-listener-btn').is(':checked')) {
      appendToConsole('Science listener not yet activated!')
    } else if ($('#activate-science-btn').is(':checked')) {
      sendScienceRequest('activate', function (msgs) {
        console.log('msgs', msgs)
        if (msgs[0]) {
          $('#activate-science-btn')[0].checked = true
          checkButtonStates()
        } else {
          $('#activate-science-btn')[0].checked = false
        }
      })
    } else {
      // 'deactivated' needs to be handled differently since it takes 45 secconds
      sendScienceRequest('stop', function (msgs) {
        console.log('msgs', msgs)
        if (msgs[0]) {
          $('#activate-science-btn')[0].checked = false
        } else {
          $('#activate-science-btn')[0].checked = true
        }
      })
    }
  })

  $('#ccw-btn').on('click', function (event) {
    if (!isScienceActivated()) {
      return
    }
    sendScienceRequest('dccw', function (msgs) {
      console.log('msgs', msgs)
      if (msgs && msgs[1].trim() == 'dccw done') {
        lightUp('#ccw-btn')
        greyOut('#cw-btn')
      }
    })
  })

  $('#cw-btn').on('click', function (event) {
    if (!isScienceActivated()) {
      return
    }
    sendScienceRequest('dcw', function (msgs) {
      console.log('msgs', msgs)
      if (msgs && msgs[1].trim() == 'dcw done') {
        greyOut('#ccw-btn')
        lightUp('#cw-btn')
      }
    })
  })

  $('#elevator-up-btn').on('click', function (event) {
    if (!isScienceActivated()) {
      return
    }
    sendScienceRequest('eup', function (msgs) {
      console.log('msgs', msgs)
      if (msgs[1].includes('eup done')) {
        lightUp('#elevator-up-btn')
        greyOut('#elevator-down-btn')
      }
    })
  })

  $('#elevator-down-btn').on('click', function (event) {
    if (!isScienceActivated()) {
      return
    }
    sendScienceRequest('edown', function (msgs) {
      console.log('msgs', msgs)
      if (msgs[1].includes('edown done')) {
        greyOut('#elevator-up-btn')
        lightUp('#elevator-down-btn')
      }
    })
  })
})

function isScienceActivated () {
  if (!$('#science-listener-btn').is(':checked')) {
    appendToConsole('Science listener not yet activated!')
    return false
  } else if (!$('#activate-science-btn').is(':checked')) {
    appendToConsole('Science MCU is not yet activated!')
    return false
  }
  return true
}

// check all button states that are query-able from the science MCU
function checkButtonStates () {
  // check if drill cw/ccw (dd = drill direction)
  sendScienceRequest('dd', function (msgs) {
    appendToConsole('dd msgs:', msgs)
    // would also check if msgs[0] was true but science MCU responds
    // with the same message too many times so it is always false
    // this is not the case with the 'activated0' response which only appears once
    if (msgs[1].includes('CCW')) {
      lightUp('#ccw-btn')
      greyOut('#cw-btn')
    } else if (msgs[1].includes('CW')) {
      console.log('true')
      lightUp('#cw-btn')
      greyOut('#ccw-btn')
    }
  })

  sendScienceRequest('ed', function (msgs) {
    appendToConsole('ed msgs:', msgs)
    if (msgs[1].includes('UP')) {
      lightUp('#elevator-up-btn')
      greyOut('#elevator-down-btn')
    } else if (msgs[1].includes('DOWN')) {
      lightUp('#elevator-down-btn')
      greyOut('#elevator-up-btn')
    }
  })
}
