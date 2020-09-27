FROM ros:melodic-ros-core

SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y ros-melodic-rosbridge-suite \
    && rm -rf /var/lib/apt/lists/* \
    && useradd --create-home --shell /bin/bash spaceuser

WORKDIR /home/spaceuser

COPY ./entrypoint.sh .
RUN chown spaceuser entrypoint.sh
USER spaceuser
RUN chmod u+x ./entrypoint.sh

ENTRYPOINT ["./entrypoint.sh"]
