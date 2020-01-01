The odroid will send an email with its local IP address after having run the ethernet config service first, and only if it does not end up selecting the `RoverOBC` configuration.

This was accomplished by running `syncEmailer.sh` and setting up a systemd startup service to run `runEmailer`.

### Dependencies

- Install node version 10.13.0 (Best way to do so is with [nvm](https://github.com/nvm-sh/nvm))
- `npm install`

### Configuration

In order to control the mailing list, add a file called `emails.txt` containing a comma-separated list of 
email addresses. This file should be placed in the same folder as the emailer script itself (`emailIPAddress.js`).