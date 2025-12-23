#!/usr/bin/env bash
set -e

NS="${1:-car_like}"

# espera controller_manager existir
for i in $(seq 1 80); do
  rosservice call "/${NS}/controller_manager/list_controllers" >/dev/null 2>&1 && break
  sleep 0.1
done

# deixa assentar
sleep 2.0

rosservice call "/${NS}/controller_manager/switch_controller" \
"{start_controllers: ['joint_state_controller',
                      'front_left_steer_controller',
                      'front_right_steer_controller',
                      'rear_left_wheel_controller',
                      'rear_right_wheel_controller'],
  stop_controllers: [],
  strictness: 2}"

