// turn table graphics
const twopi = 2 * Math.PI
const pi = Math.PI

run = async () => {
  const canvas = document.getElementById('turntable-canvas')
  const w = canvas.width
  const h = canvas.height
  const context = canvas.getContext('2d')

  const x = 0.8 // at x=1 circle touches canvas edge
  const r = x * 0.5 * Math.min(w, h)

  // BEGIN: GET WHEEL PARAMS FROM SERVER
  const response1 = await fetch('http://localhost:5000/science/numSections')
  const n = Number(await response1.text())

  const response2 = await fetch('http://localhost:5000/science/initialSection')
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
    console.log('rotatePos')
    button_rpos.disabled = true
    button_rneg.disabled = true

    const start = (((-i + n) % n) * twopi) / n
    for (let [a, z] = [start, 0]; z < nframes; [a, z] = [a - d, z + 1]) {
      await timer(1)
      drawWheel(context, w, h, r, n, a)
    }

    i = (i + 1) % n
    drawWheel(context, w, h, r, n, (-i * twopi) / n)

    button_rpos.disabled = false
    button_rneg.disabled = false
    lastRotation = Date.now()
  }

  const button_rneg = document.getElementById('button_rneg')
  window.rotateNeg = async () => {
    console.log('rotateNeg')
    button_rpos.disabled = true
    button_rneg.disabled = true

    const start = (-i * twopi) / n
    for (let [a, z] = [start, 0]; z < nframes; [a, z] = [a + d, z + 1]) {
      await timer(1)
      drawWheel(context, w, h, r, n, a)
    }

    i = (i + n - 1) % n
    drawWheel(context, w, h, r, n, (-i * twopi) / n)

    button_rpos.disabled = false
    button_rneg.disabled = false
    lastRotation = Date.now()
  }

  $('#button_rpos').on('click', function (event) {
    if (!isScienceActivated()) {
      return
    }
    sendRequest('Science', 'tccwstep', function (msgs) {
      console.log('msgs', msgs)
    })
  })

  $('#button_rneg').on('click', function (event) {
    if (!isScienceActivated()) {
      return
    }
    sendRequest('Science', 'tcwstep', function (msgs) {
      console.log('msgs', msgs)
    })
  })
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
  let indices = [0, 3, 2, 1]
  for (let z = 0; z < n; z++) {
    let number = indices[z]
    let a_ = a - z * (twopi / n)
    let x = Math.cos(a_) * r * 0.8
    let y = Math.sin(a_) * r * 0.8
    ctx.fillText(`${number}`, mx + x, my + y)
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

$(document).ready(() => {
  run()
  $('#turntable-cw-btn').on('click', function (event) {
    if (!isScienceActivated()) {
      return
    }
    sendRequest('Science', 'tcw', function (msgs) {
      console.log('msgs', msgs)
    })
  })

  $('#turntable-ccw-btn').on('click', function (event) {
    if (!isScienceActivated()) {
      return
    }
    sendRequest('Science', 'tccw', function (msgs) {
      console.log('msgs', msgs)
    })
  })

  $('#turntable-stop-btn').on('click', function (event) {
    if (!isScienceActivated()) {
      return
    }
    sendRequest('Science', 'ts', function (msgs) {
      console.log('msgs', msgs)
    })
  })
})
