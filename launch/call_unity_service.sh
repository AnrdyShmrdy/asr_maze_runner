#!/bin/bash
rosservice call --wait /spawn_unity_robot ponce &
exec "$@"