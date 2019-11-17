function toggleText (button) {
  if (button.text() == button.data('text-swap')) {
    button.text(button.data('text-original'))
  } else {
    button.data('text-original', button.text())
    button.text(button.data('text-swap'))
  }
}
