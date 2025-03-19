#!/bin/sh

while true; do (speaker-test -D hw:1,1 -c2 & sleep 0.1; kill $!); sleep 0.1; done
