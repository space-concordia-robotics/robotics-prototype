function keepText (button) {
  if (window.localStorage.getItem(button) == 'text-swap') {
    toggleText(button)
  }
}

function toggleText (button) {
  if (button.text() == button.data('text-swap')) {
    button.text(button.data('text-original'))
    window.localStorage.setItem(button, 'text-original')
  } else {
    button.data('text-original', button.text())
    button.text(button.data('text-swap'))
    window.localStorage.setItem(button, 'text-swap')
  }
}
