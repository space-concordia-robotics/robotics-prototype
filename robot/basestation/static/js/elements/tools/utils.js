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
