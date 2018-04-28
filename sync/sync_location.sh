
#!/bin/bash

pwd=$PWD
pi_ip="192.168.4.1"
rsync -avzh $pwd/test.txt pi@$pi_ip:~/drone/test.txt
