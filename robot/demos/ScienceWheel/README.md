## How to run
`python3 server.py`
Then, visit `localhost:8000`

## What you get
A nice gui that visualises the rotation of a disc divided into `n` sections. When you click 'Rotate', it calls a server which sleeps for 1 second before responding, to simulate the delay of the physical disk rotation.

`n` is a parameter defined in `server.py`, as is `initialSection` and the delay time

## Dependencies
* `python3`
* `flask`
* browser that supports the `fetch` API

Creds to @stumash
