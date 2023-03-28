FROM osrf/ros:humble-desktop-full
LABEL Name=rostest Version=0.0.1

COPY apt-dependencies.txt /tmp/apt-dependencies.txt
RUN apt-get update && xargs -a /tmp/apt-dependencies.txt apt-get install -y

COPY pip-dependencies.txt /tmp/pip-dependencies.txt
RUN pip install -r /tmp/pip-dependencies.txt

# setup entrypoint
COPY ./entrypoint.sh ./entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["./entrypoint.sh"]