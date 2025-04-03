'''
This code was developed by Andrew Curtis in the year 2024.
It was developed at Northwestern University, Evanston, IL, USA.

This serves as interface with the Flight Controller Board for each quadrotor in the QuadSwarm.
From here, each robot sends commands to the flight controller board and receives information from the flight controller board
'''

#import 3rd party libraries
from yamspy import MSPy

#import os files (files NU created)
from lib.shared_data_management import SharedDataManager

#import native libraries
import time
import numpy as np
from itertools import cycle
import traceback


np.seterr(all='raise')

class FlightControllerInterface():
    '''
    A class to handle the interface with the flight controller
    It is intialized by bootloader before it's 'run' task is kicked off as a process in the multiprocess setup
    '''
    def __init__(self, id, conn) -> None:
        '''
        gets a unique robot id (from bootloader when the fc handler object is created) and stores it

        Also gets a pipe connection to bootloader so it can pass info for logging.

        Initializes the commands and battery management values
        '''
        self.id = id
        self.bootloader_pipe_connection = conn

        #set up variables for communicating with the board.
        self.CMDS = {
                'roll':     1500,
                'pitch':    1500,
                'throttle': 900,
                'yaw':      1500,
                'aux1':     1000, #1800 is arm
                'aux2':     1500
                }
        
        #aux 1 modes:
        # 1000 == DISARM
        # 1800 == ARM

        #aux 2 modes:
        # 1500 == horizon
        # 1000 == angle
        # 2000 == Flip

        # This order is the important bit: it will depend on how your flight controller is configured.
        # Below it is considering the flight controller is set to use AETR.
        # The names here don't really matter, they just need to match what is used for the CMDS dictionary.
        # In the documentation, iNAV uses CH5, CH6, etc while Betaflight goes AUX1, AUX2...
        self.CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']
        self.slow_msgs = cycle(['MSP_ANALOG']) #cycle(['MSP_ANALOG', 'MSP_STATUS_EX', 'MSP_MOTOR', 'MSP_RC'])
        self.SERIAL_PORT = "/dev/serial0"
        self.ARMED = False
        self.CMDS_FREQ = 0.0
        self.SLOW_MSGS_LOOP_TIME = 1/5 # these messages take a lot of time slowing down the loop...
        self.last_slow_msg_time = time.time()
        
        #Battery management
        self.voltage = -1.0 #initialize to -1 so we know the voltage has been read (it'll read a positive value)
        self.amperage = 0.0
        self.power = 0.0
        self.mAhdrawn = 0.0

        #TODO make these a readable parameter from some input file
        #This is the threshold under which a low voltage immediate controlled landing is initiated.
        self.low_voltage_threshold = 6.4 #Volts
        self.low_voltage_threshold_timer = 3 #s (if the battery voltage is below the threshold for this many seconds, we will trigger an auto-land)
        self.last_time_above_threshold = time.time()

        #controller limits
        self.max_diff = 50
        self.max_command = 1950
        self.min_command = 900

    def run(self, shared_data):
        '''
        Listen for incoming commands from the shared information
        Communicate those commands to the flight controller board

        Listen to flight controller for feedback on battery voltage, etc.
        '''

        data_manager = SharedDataManager(shared_data)

        self.bootloader_pipe_connection.send('fc handler trying to connect')

        try:
            with MSPy(device=self.SERIAL_PORT, loglevel='WARNING', baudrate=115200) as board:
                # if board == 1 or board.CONFIG['flightControllerIdentifier'] == '':
                #     self.bootloader_pipe_connection.send('There was an error connecting to the FC')
                #     time.sleep(0.5) #need to sleep so it can be logged.
                #     return 1
                
                #Flag that we have connected with the board, so the watch dog dimer doesn't go off.
                data_manager.set_board_connected(1)

                # It's necessary to send some messages or the RX failsafe will be activated
                # and it will not be possible to arm.
                #here we build that initial command list
                command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO', 
                                'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
                                'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES']
                if board.INAV:
                    command_list.append('MSPV2_INAV_ANALOG')
                    command_list.append('MSP_VOLTAGE_METER_CONFIG')

                #send initial commands to prevent the RX failsafe from activating
                for msg in command_list: 
                    if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)
                
                data_manager.set_safety(5) #let the rest of the processes know we are trying to connect to the fc board by sending disarm for Xs

                #send a disarm command for Xs as the drone gets warmed up
                self.initial_cmds(5, board) #1s #TODO make this known to the bootloader, etc. so code isn't started during this time.

                if data_manager.get_safety() == 5: #just double check that no one else overwrote the safety value before we set it back to zero.
                    data_manager.set_safety(0)
                



                while True:


                    #get commands from shared data. If nothing is there, returns success = False, so we won't
                    #process them (i.e., we won't write them into the CMDS array)
                    commands, success = data_manager.get_commands(clear_fresh_flag=True, blocking=False)
                    if success:
                        #process them into the dictionary values
                        #also checks to make sure they are safe for the flight controller
                        self._process_cmds(commands)


                    #check the safety codes. if 1, that's an emergency stop and we need to reset the board to kill everything.
                    #if 2, it's non-emergency, but we are going to shut down anyway.
                    safety = data_manager.get_safety()
                    if safety == 1 or safety == 2:
                        board.reboot()
                        self.bootloader_pipe_connection.send('The flight controller has responded to an emergency stop code %d. The board has been reset.' % safety)
                        time.sleep(0.5) #this gives the board a moment to reset and the logger a moment to log.
                        data_manager.set_safety(0) #the safety is non-zero, so we have to make it zero in order to set it to 5.
                        data_manager.set_safety(5) #We do this, so we won't be able to start a new user code until the FC has connected again.
                        return
                    
                    # self.CMDS['aux1'] = 1800
                    # self.CMDS['throttle'] = 900
                    # self.CMDS['roll'] = 1500
                    # self.CMDS['pitch'] = 1500
                    # self.CMDS['yaw'] = 1500
                    # self.CMDS['aux2'] = 1500

                    #Push controls to FC
                    # Send the RC channel values to the FC
                    if board.send_RAW_RC([self.CMDS[ki] for ki in self.CMDS_ORDER]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)

                    #
                    # Communicate slow messages with the FC 
                    # SLOW MSG processing (user GUI)
                    #
                    if (time.time()-self.last_slow_msg_time) >= self.SLOW_MSGS_LOOP_TIME:
                        self.last_slow_msg_time = time.time()

                        next_msg = next(self.slow_msgs) # circular list

                        # Read info from the FC
                        if board.send_RAW_msg(MSPy.MSPCodes[next_msg], data=[]):
                            dataHandler = board.receive_msg()
                            board.process_recv_data(dataHandler)
                            
                        #get the battery info if that's the message we inquired about
                        if next_msg == 'MSP_ANALOG':
                            self.voltage = board.ANALOG['voltage']
                            self.amperage = board.ANALOG['amperage']
                            self.mAhdrawn = board.ANALOG['mAhdrawn']
                            self.power = self.amperage * self.voltage 

                            data_manager.set_battery_voltage(self.voltage)
                            data_manager.set_battery_power(self.power)

                            #check the battery voltage
                            if self.voltage < self.low_voltage_threshold:
                                #If more than the low_voltage_threshold_timer time as passed since we were last above the threshold,
                                #then we will initialize a safety code 30
                                if self.last_slow_msg_time - self.last_time_above_threshold >= self.low_voltage_threshold_timer:
                                    data_manager.set_safety(4)
                                    self.bootloader_pipe_connection.send('LOW VOLTAGE WARNING!! ' + str(self.voltage))
                                    data_manager.set_low_voltage_warning()
                            else:
                                #if not below threshold, reset the last time we saw a voltage above the threshold
                                self.last_time_above_threshold = self.last_slow_msg_time
        except Exception as error:

            #log the error and continue
            logMessage = ['an error occurred']
            trace = traceback.extract_tb(error.__traceback__)
            for t in trace:
                logMessage.append(str(t))
            logMessage.append([str(type(error).__name__)])
            logMessage.append([str(error)])
            self.bootloader_pipe_connection.send(logMessage) #send the error to the bootloader to be logged
            return


    def exit(self):
        pass

    def _process_cmds(self, commands):
        '''
        Accepts an array of commands and stores them in the CMDS dictionary.
        Input array: AETR (roll, pitch, throttle, yaw), ARM, and MODE
        '''

        #First, make sure each AETR command is safe
        for i,c in enumerate(commands[0:4]):
            if i == 0:
                key = 'roll'
            elif i == 1:
                key = 'pitch'
            elif i == 2:
                key = 'throttle'
            elif i == 3:
                key = 'yaw'
            new_c = self._fc_protector(c, self.CMDS[key])
            commands[i] = new_c

        #Then write the commands into the CMDS dictionary so it can go to the fc board.
        self.CMDS['roll'] = commands[0]
        self.CMDS['pitch'] = commands[1]
        self.CMDS['throttle'] = commands[2]
        self.CMDS['yaw'] = commands[3]
        self.CMDS['aux1'] = commands[4]
        self.CMDS['aux2'] = commands[5]

    def _fc_protector(self, control_value, old_value):
        '''
        Compares a new control value w. the old value. Makes sure it's within max diff and within max/min absolutes.
        Returns the updated (bounded) value if necessary
        '''
        try:
            #First, make sure the jump isn't too big.
            control_value = int(control_value)
            old_value = int(old_value)
            if abs(control_value - old_value) > self.max_diff:
                if control_value < old_value:
                    control_value = old_value - (self.max_diff)
                elif control_value > old_value:
                    control_value = old_value + (self.max_diff)

            #then make sure its within the max-min command range.
            if control_value > self.max_command:
                return self.max_command
            elif control_value < self.min_command:
                return self.min_command
            else:
                return control_value
            
        except:
            #'there was a run time warning'
            #happens if calculation fails (some overflow or something due to large numbers that shouldn't be (or dvide by zero or something))
            #default to a 1500 return value as a safety.
            return 1500


    def initial_cmds(self, X, board):
        '''
        The initial disarm commands while the drone boots up.
        '''
        start_time = time.time()
        start_up_time = X #s
        while time.time() < start_time + start_up_time:
            self.CMDS['aux1'] = 1000 #Send disarm command.

            self.CMDS['aux2'] = 1500
            #aux 2 modes:
            # 1500 == horizon
            # 1000 == angle
            # 2000 = Flip

            #AETR are default/min
            self.CMDS['throttle'] = 900
            self.CMDS['roll'] = 1500
            self.CMDS['pitch'] = 1500
            self.CMDS['yaw'] = 1500

            #Push controls to FC
            # Send the RC channel values to the FC
            if board.send_RAW_RC([self.CMDS[ki] for ki in self.CMDS_ORDER]):
                dataHandler = board.receive_msg()
                board.process_recv_data(dataHandler)

    def foobar(self, X, board):
        '''
        The initial disarm commands while the drone boots up.
        '''
        start_time = time.time()
        start_up_time = X #s
        while time.time() < start_time + start_up_time:
            self.CMDS['aux1'] = 1800 #Send ARM command.

            self.CMDS['aux2'] = 1500
            #aux 2 modes:
            # 1500 == horizon
            # 1000 == angle
            # 2000 = Flip

            #AETR are default/min
            self.CMDS['throttle'] = 900
            self.CMDS['roll'] = 1500
            self.CMDS['pitch'] = 1500
            self.CMDS['yaw'] = 1500

            #Push controls to FC
            # Send the RC channel values to the FC
            if board.send_RAW_RC([self.CMDS[ki] for ki in self.CMDS_ORDER]):
                dataHandler = board.receive_msg()
                board.process_recv_data(dataHandler)
        