#!/bin/bash

arecord -f S16_LE -r 48000 -D hw:TW6869,0,1 | aplay
