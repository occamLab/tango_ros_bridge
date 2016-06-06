#!/usr/bin/env python

from udp import UDPhandle

port = 11118

start_delim = "RSSISTART\n"
end_delim = "RSSIEND\n"



@UDPhandle(port=port, start_delim=start_delim, end_delim=end_delim)
def handle_pkt(pkt=None):
    f = open('rssi_log.txt', 'a')
    #f.seek(-1, 2)
    f.write(str(pkt) + '\n')
    f.close()

handle_pkt()
