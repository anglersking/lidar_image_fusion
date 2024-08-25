# Raw Robosense LIDAR packet.

time stamp              # packet timestamp
int16 id                # packet id. start from 1
uint8[80] header        # packet header
uint8[1168] data        # packet contents

