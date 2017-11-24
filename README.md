# robotics-prototype
This repo contains the beginning of the new (2017-2018) robotics software team code

#### Software Team Lead:
- Peter [@PeterGhimself](https://github.com/PeterGhimself)

#### Lidar Team:
- David [@Davidster](https://github.com/Davidster)
- Peter [@PeterGhimself](https://github.com/PeterGhimself)
- Samuel [@samuelbeaubien](https://github.com/samuelbeaubien)

#### Communications Team:
- Alex [@alexandertoutounghi](https://github.com/alexandertoutounghi)
- Peter [@PeterGhimself](https://github.com/PeterGhimself)

#### Motor-Interface Team:
- Kevin [@kcamcam](https://github.com/kcamcam)
- Josh [@CraniumField](https://github.com/CraniumField)

#### GUI Team:
- Beeri [@bnduwi](https://github.com/bnduwi)
- Zayn [@ZaynMuhammad](https://github.com/ZaynMuhammad)
- Line [@LineG](https://github.com/LineG)

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

