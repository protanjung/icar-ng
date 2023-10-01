#!/usr/bin/bash

# This command uses colcon to build the project and create symbolic links to the installed files.
colcon --log-base /dev/null build --symlink-install --event-handlers desktop_notification-