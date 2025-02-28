#!/bin/sh

while true; do (arecord -v -f dat -D hw:1,0 -c2 - > /dev/null & sleep 0.1; kill $!); sleep 0.1; done
