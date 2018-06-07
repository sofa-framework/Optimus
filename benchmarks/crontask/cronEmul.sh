#!/bin/bash

while :; do
   currenttime=$(date +%H:%M)
   if [[ "$currenttime" > "12:01" ]] && [[ "$currenttime" < "12:07" ]]; then
     /bin/bash /home/sergei/Optimus_test/runOptimusTests.sh >> /home/sergei/Optimus_test/log_`/bin/date +"%d_%m_%Y"`.txt
     sleep 360s
   else
     sleep 50s
   fi
done &

# cron command
# 00 12 * * 1-5 /home/sergei/Optimus_test/runOptimusTests.sh >> /home/sergei/Optimus_test/log_`/bin/date +"\%d_\%m_\%Y"`.txt

# rc.local command
# exec setuidgid sergei /bin/bash /home/sergei/Optimus_test/cronEmul.sh
