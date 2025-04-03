import socket
import os
import time
import numpy as np
import struct

class Bot():
    def __init__(self, id) -> None:
        self.id = id
        self.state = 'off'
        self.voltage = 0
        self.last_command = 'none'
        self.last_update_time = time.time()
        self.safety = 0
        self.user_code = 'none'

NUM_BOTS = 50
BOT_START_NUM = 5
TIMEOUT = 10 #s

def main():

    try: 
        port = 60402
        #This creates a UDP socket 
        recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #This allows the socket address to be reused (which I think helps with the sender socket above)
        recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        #We're going to listen to anything coming over the port on the localhost network
        recv_socket.bind(('localhost', port))
        recv_socket.setblocking(1) #0.5 second timeout
        recv_socket.settimeout(0.5)

        #create all of the robots that we will be tracking
        bots = []
        for i in range(BOT_START_NUM, BOT_START_NUM+NUM_BOTS):
            bot = Bot(i)
            bots.append(bot)

        #lists to help us keep track of data that needs updating, etc.
        data_to_update = {}
        bots_to_update = []
        has_been_on = []

        #optitrack data
        num_bots = 0
        opti_loss = 0
        optitime = 0
        heartbeat = 0
        count_diff = 0
        avg_time_between = 0.0

        reprint = True #used so we only print when we get new robot data (otherwise print too fast b/c of optitrack data)
        time_of_last_print = 0
        time_between_prints = 0.5
        reprint_debounce_time = 0.3

        opti_udp = OptitrackInterfaceUDP()

        while True:

            timeout_bots = []

            if reprint and time.time() - time_of_last_print <= reprint_debounce_time:
                reprint = False

            if reprint:
                heartbeat += 1
                if heartbeat > 100:
                    heartbeat = 0
                time_of_last_print = time.time()

                os.system('clear')
                print('ID    Safety\tState\t\tLast CMD\tVoltage\t\tUsrCode')
                now = time.time()
                for i,bot in enumerate(bots):
                    if bot.id in bots_to_update:
                        bot.voltage = data_to_update[bot.id][0]
                        bot.state = data_to_update[bot.id][1]
                        bot.last_command = data_to_update[bot.id][2]
                        bot.safety = data_to_update[bot.id][3]
                        bot.user_code = data_to_update[bot.id][4]
                        bot.last_update_time = now

                    #keep track of any bots that may have timed out (we haven't heard from in forever.)
                    if bot.id in has_been_on:
                        if now >= bot.last_update_time + TIMEOUT:
                            bot.voltage = 0
                            bot.state = 'off'
                            bot.last_command = 'none'
                            bot.safety = 0
                            bot.user_code = 'none'
                            timeout_bots.append(bot.id)


                    

                    id = str(bot.id).zfill(2)
                    print("%s\t%d\t%s\t\t%s\t\t%0.2f\t\t%s" % (id, bot.safety, bot.state, bot.last_command, bot.voltage, bot.user_code))

                print('\n')
                print('Optitrack Info:')
                print('Bodies: %d\tLR: %0.2f\t Missed Counts: %d\tHB: %d' %(num_bots, opti_loss, count_diff, heartbeat))
                print('Time btwn: %0.3f' %(avg_time_between), '\tTimeout bots: ' + str(timeout_bots))


                reprint = False
            else:
                if time.time() - time_of_last_print > time_between_prints:
                    reprint = True


            #get a list of messages (unpack the raw data)
            try:
                data_to_update = {}
                bots_to_update = []

                raw_recv_data = recv_socket.recv(1024)
                recv_data = raw_recv_data.decode('utf-8')
                data = process_inbound(recv_data)

                if data == False:
                    pass
                else:
                    #parse the messages and record what needs to be updated
                    #message in the order: id, volts, safety code, state, last command, user code file
                    for msg in data:
                        
                        sender = int(msg[0])
                        volts = float(msg[1])
                        safety = int(msg[2])
                        state = msg[3]
                        last_command = msg[4]
                        user_code = msg[5]
                        data_to_update[sender] = [volts, state, last_command, safety, user_code]
                        bots_to_update.append(sender)
                        if sender not in has_been_on:
                            has_been_on.append(sender)
                        if state == 'off' and sender in has_been_on:
                            has_been_on.pop(has_been_on.index(sender))

                        reprint = True
    
            except BlockingIOError:
                # print('timeout')
                pass
            except TimeoutError:
                pass

            # print('checking optitrack')

 
            #NOW GET DATA FROM OPTITRACK!!!
            opti_data = opti_udp.run()
            # print(opti_data)
            if opti_data == False:
                pass
            else:
                # print(opti_data)
                num_bots = opti_data[0]
                opti_loss = opti_data[1]
                count_diff = opti_data[2]
                avg_time_between = opti_data[3]
                reprint = True


    except KeyboardInterrupt:
        print('User interrupted.')
        os.system('clear')
        
    finally:
        pass
        


def process_inbound(recv_data):
    '''
    Processes the received data into messages using headers and message length info
    returns a list of all messges received from that particular IP address.
    returns FALSE if failure
    '''
    msgs = []
    msg_start_indexes = []
    msg_end_indexes = []

    try:
        offset = 2 #where the actual message starts w.r.t the 'abc' value
        split_message = recv_data.split(',')

        #This bit gives us each message from the client (in case there were multiple. Usually only one)
        for i, part in enumerate(split_message):
            if part == 'abc':
                msg_start_indexes.append(i+offset)
                msg_len = int(split_message[i+1]) #get the length of the message (stored after abc)
                msg_end_indexes.append(i+offset+msg_len)

        #This loops through each message and process it so its ready to be returned.
        for i,index in enumerate(msg_start_indexes):
            msgs.append(split_message[index:msg_end_indexes[i]])
                        
        if len(msgs) > 0:
            return msgs
        else:
            return False
    except:
        return False


class OptitrackInterfaceUDP():
    '''
    Class to interface with optitrack and get data from the optitrack machine using UDP
    '''
    def __init__(self, id = 2, ip="224.1.1.1", port=54321) -> None:
        '''
        gets a unique robot id (default id = 2 for the air traffic controller) and stores it

        Creates a udp client socket to listen for information from the optitrack computer. 
        DEVELOPERS NOTE: Port and ip address are for the optitrack computer (which acts as a server.)
        '''       

        self.port = port
        self.ip = ip #IP address of MOTIVE machine

        #set up a socket to receive messages from the server.
        #AF_INET is the internet address family for IPv4
        #SOCK_DGRAM is the socket type for the UDP protocol.
        #socket.IPPROTO_UDP is also for the UDP protocol (to remove any ambiguity.)
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

        #This allows the socket address to be reused (which I think helps with the sender socket above)
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        #REUSE THE PORT TOO (ONLY ON LINUX)
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

        #Increase buffer size
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 131072)

        #We're going to listen to anything coming over the port from the ip address given
        self.client_socket.bind(('', self.port))

        # Join the multicast group
        mreq = struct.pack("4sL", socket.inet_aton(self.ip), socket.INADDR_ANY)
        self.client_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        #make the socket blocking
        self.client_socket.setblocking(False)
        

        self.id = id

        self.last_opti_count = 0
        self.lost_message_tracker = np.zeros(1000) #100 == 1s of data, so 1000 is 10s
        self.lost_message_pointer = 0
        self.first_data = True

        self.avg_time_between = 0.0
        self.alpha = 0.9
        self.last_time_found = time.time()
    
    def run(self):
        '''
        listen for incoming information from the optitrack system
        calculate and update shared memory data with new optitrack information.
        '''
        try:
            recv_data, addr = self.client_socket.recvfrom(1024) #data will return already decoded, but not split
            data_length = len(recv_data)

            log_no_data = False #set this to True if you want to log that data was missed for this particular loop 
            error_msg = 'normal' #set this to identify the reason data was lost.

            #if we got data to parse
            if recv_data:

                self.__time_of_last_receipt = time.time()

                #We assume we only have 1 message per 1024 bytes pulled from the receive buffer.
                #We quickly check this by looking at the length of the data. We expect each message to be less than 800 bytes long (29bytes*25drones=725)
                #if we only have 1 message, we know 'optiX' should be at the beginning, 
                #and we are looking for 'optiX' because that is the identifier at the beginning of each optitrack message
                ##where 'optiX' could be 'opti1' for IDs 5-29 and 'opti2' for IDs 30-54
                #if we have more than 1 message, we loop through until we find 'optiX'

                #messages are in the format: 'opti', timestamp, sequence number, id, x, y, z, q1, q2, q3, q4, id, x, y, z, q1, q2, q3, q4,
                #                                                                 0, 1, 2, 3,  4,  5,  6,  7,

                data_found = False
                message_start = [] #will be used to hold the indexes of the beginning of the message
                if data_length < 800:
                    segment = struct.unpack('5s', recv_data[0:5])[0]
                    #try to turn the segment of the message into a string. If you can't do that (UnicodeDecodeError, just continue to the next message)
                    try:
                        segment = segment.decode('utf-8')
                    except UnicodeDecodeError:
                        return False

                    if segment == 'opti1' or segment == 'opti2':
                        preamble = struct.unpack('5sfH', recv_data[0:14])
                        optitime = preamble[1]
                        opticount = preamble[2]
                        #Make sure the sequence number is not an old one.
                        #and check the roll over at 65535 (if we miss more than 200 messages, we are screwed anyway)
                        roll_over_flag = False
                        if opticount < 0+100 and self.last_opti_count > 65535 - 100:
                            roll_over_flag = True
                        if opticount <= self.last_opti_count and not roll_over_flag:
                            data_found = False
                            return False
                        time_found = time.time()
                        data_found = True
                else:
                    print('data too long!!')
                
                
                if data_found:
                    if self.first_data:
                        percent_loss = 0.0
                        self.first_data = False
                        msg_count_diff = 1
                    else:

                        msg_count_diff = int(opticount - self.last_opti_count)%65536
                        #if the difference is one, we missed no messages
                        if msg_count_diff == 1:
                            self.lost_message_tracker[self.lost_message_pointer] = 0
                            self.lost_message_pointer += 1
                            if self.lost_message_pointer >= len(self.lost_message_tracker):
                                self.lost_message_pointer = 0
                        else:
                            #if the difference is greater than one, we missed some messages and need to update our tracker accordingly
                            for i in range(0,msg_count_diff):
                                if i == msg_count_diff - 1:
                                    self.lost_message_tracker[self.lost_message_pointer] = 0
                                else:
                                    self.lost_message_tracker[self.lost_message_pointer] = 1
                                
                                self.lost_message_pointer += 1
                                if self.lost_message_pointer >= len(self.lost_message_tracker):
                                    self.lost_message_pointer = 0
                        
                        percent_loss = (np.sum(self.lost_message_tracker)/len(self.lost_message_tracker))*100

                    time_difference = time_found - self.last_time_found
                    self.avg_time_between = self.alpha * time_difference + (1-self.alpha) * self.avg_time_between

                    #estimate number of bodies by message size
                    num_bodies = int((data_length-14)/29) #preamble is 14 bytes, and each robot is 29 bytes
                    num_bodies = 42

                    self.last_opti_count = opticount
                    self.last_time_found = time_found
                    return [num_bodies, percent_loss, msg_count_diff-1, self.avg_time_between]
                
        except BlockingIOError:
            # print('blocking error')
            return False


        # num_bots = opti_data[0]
        # opti_loss = opti_data[1]
        # count_diff = opti_data[2]












main()