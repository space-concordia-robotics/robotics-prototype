## Usage

Note: `docker-compose up` and `docker-compose down` need to be prefixed by `sudo` unless you are part of the `docker` group.
See [here](https://docs.docker.com/engine/install/linux-postinstall/) for more information.

* First you need to install [docker](https://docs.docker.com/engine/install/ubuntu/) and [docker-compose](https://docs.docker.com/compose/install/)
* `cd` into the `docker` directory
* `docker-compose up`. The first time you run this it will take a while (install dependencies, build etc).
* Once all services are up, visit http://localhost:5000
* When you're done, tear everything down with `docker-compose down` (in the `docker` directory as well)
