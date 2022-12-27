#!/bin/bash

rm lm75a.dtbo
sleep 1
dtc -I dts -O dtb -o lm75a.dtbo lm75a-overlay.dts
