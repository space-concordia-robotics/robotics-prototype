FROM ros:melodic-ros-core

SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y ros-melodic-rosbridge-suite \
    && rm -rf /var/lib/apt/lists/* 

COPY ./entrypoint.sh .

RUN chmod u+x ./entrypoint.sh

ENTRYPOINT ["./entrypoint.sh"]
