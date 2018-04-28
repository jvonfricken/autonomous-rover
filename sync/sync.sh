#!/usr/bin/env bash

pwd=$PWD

pi_ip="192.168.1.107"
rsync -avzh $pwd/../ pi@$pi_ip:~/drone/
