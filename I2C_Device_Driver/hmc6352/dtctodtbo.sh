#!/bin/bash

rm hmc6352.dtbo
sleep 1
dtc -I dts -O dtb -o hmc6352.dtbo hmc6352-overlay.dts
