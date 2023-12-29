#!/bin/bash

./pid_controller/pid_controller 0.12 0.08 0.0 0.3 0 0 &
sleep 1.0
python3 simulatorAPI.py
