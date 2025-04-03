#!/usr/bin/env python3
import socket
import time
import struct

def main():
    # Create a UDP socket and enable broadcasting
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    sequence_number = 0

    while True:
        # Use a constant float value 0.0 so its binary representation is all zeros.
        constant_time = 0.0

        # Build a 14-byte preamble:
        #   5s   => 5-byte string: b'opti1'
        #   f    => 4-byte float: constant_time (0.0 → b'\x00\x00\x00\x00')
        #   H    => 2-byte unsigned short: sequence number
        preamble = struct.pack('5sfH', b'opti1', constant_time, sequence_number)

        # The listener/localize code expects 29 bytes per “body.”
        # We'll send 29 dummy bytes.
        body = b'\x00' * 29

        # Combine preamble and body into one packet
        packet = preamble + body

        # Print debug info
        print(f"[DEBUG] Sending packet: opti1, constant_time=0.0, seq={sequence_number}, size={len(packet)} bytes")

        # Broadcast the packet to port 54321 on your local subnet
        sock.sendto(packet, ('<broadcast>', 54321))

        # Increment the sequence number (wrap at 65535)
        sequence_number = (sequence_number + 1) % 65536

        # Pause for 0.1 seconds (~10 packets per second)
        time.sleep(0.1)

if __name__ == '__main__':
    main()
