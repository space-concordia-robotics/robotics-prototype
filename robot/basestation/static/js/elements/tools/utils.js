// Applies modulo and if the value is negative, cycle it.
function negativeModulo (number, modulo) {
  while (number < 0) {
    number += modulo
  }

  number %= modulo
  return number;
}

// Rotate element to given angle
function rotateElement (element, angle) {
  element.css({
    transform: 'rotate(' + angle + 'deg)',
    '-ms-transform': 'rotate(' + angle + 'deg)',
    '-moz-transform': 'rotate(' + angle + 'deg)',
    '-webkit-transform': 'rotate(' + angle + 'deg)',
    '-o-transform': 'rotate(' + angle + 'deg)'
  })
}

/**
 * Clamps a value in a given range.
 *
 * @param {number} number The number to clamp.
 * @param {number} [lower] The lower bound.
 * @param {number} upper The upper bound.
 * @returns {number} Returns the clamped number.
 */
function clamp(number, lower, upper) {
  if (number === number) {
    if (upper !== undefined) {
      number = number <= upper ? number : upper;
    }
    if (lower !== undefined) {
      number = number >= lower ? number : lower;
    }
  }
  return number;
}

function getDeviceNameByMuxID (id) {
  switch(id) {
    case 'mux-0':
      return 'Rover'
      break;
    case 'mux-1':
      return 'Arm'
      break;
    case 'mux-2':
      return 'Science'
      break
  }
}
