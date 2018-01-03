# robotics-prototype
This repo contains the beginning of the new (2017-2018) robotics software team code

## How to upload arduino scripts from the odroid

1. Make sure the arduino is plugged into the odroid

2. Copy your arduino source(s) into platformio/src/

3. Navigate to platformio/ folder

4. Upload the script via the following command: `platformio run -t upload`. This will both compile and upload the code.

Note: I didn't look into adding libraries yet but I'm pretty sure you want to place them in the platformio/lib folder. See [platformio lib help page](http://docs.platformio.org/en/latest/userguide/lib/index.html) to learn more

## odroid ip emailer service

The odroid will send an email with its local IP address every time it boots

This was accomplished by running `syncEmailer.sh` and adding the following line to the crontab via `crontab -e`:

```
@reboot /home/odroid/emailer/runEmailer.sh
```

Let Peter or David know if you want to be added to this mailing list.

### to connect to the odroid from home

- Open a terminal (I recommend git bash if you're using windows)
- SSH into Concordia's network with your netname (type the following into the terminal): 
```
ssh net_name@login.encs.concordia.ca
```
- It should ask you for a password, which will your ENCS password
- Grab the latest IP address of the odroid from your email, then ssh into it: 
```
ssh odroid@ip_address
```
- It should ask you for a password, which will be `odroid`

