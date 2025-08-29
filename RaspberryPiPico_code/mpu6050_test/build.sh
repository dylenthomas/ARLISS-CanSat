#!/bin/bash
/home/dylenthomas/.pico-sdk/picotool/2.1.1/picotool/picotool reboot -f -u && sleep 1
find ./ -name "*.uf2" -execdir /home/dylenthomas/.pico-sdk/picotool/2.1.1/picotool/picotool load "{}" \; && sleep 1
/home/dylenthomas/.pico-sdk/picotool/2.1.1/picotool/picotool reboot -f -a