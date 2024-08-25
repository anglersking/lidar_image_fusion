#!/bin/bash
while [ true ]
do
current=`date "+%Y-%m-%d %H:%M:%S"`  
timeStamp=`date -d "$current" +%s`   
currentTimeStamp=$((timeStamp*1000+`date "+10#%N"`/1000000)) #将current转换为时间戳，精确到毫秒  
echo $currentTimeStamp' ms'

done
