import time
import traceback
import json
import os
import numpy as np
import csv
from subprocess import check_output
import importlib
import struct
import multiprocessing as mp

# QuadSwarm modules:
import localize
import fc_handler
import control_manager
import user_code_handler
import logbook
import comms
import lib.shared_data_management
import lib.bootloader_stm

MAX_TRYS = 1

def main(voltage_status):
    print("DEBUG: Entering main() without ATC. voltage_status =", voltage_status)
    ret_val = 'normal'
    GLOBAL_START_TIME = time.time()

    def now():
        return format(np.round(time.time() - GLOBAL_START_TIME, 3), '0.3f')

    # Acquire IP address
    self_ip = ""
    ip_flag = False
    robot_id = 99
    while not ip_flag:
        time.sleep(0.5)
        try:
            byte_ip = check_output(['hostname', '-I'])
            raw_ip = byte_ip.decode('utf-8')
            ip_candidates = raw_ip.strip().split()
            for token in ip_candidates:
                # e.g., "10.0.0." for your local net
                if token.startswith("10.0.0."):
                    self_ip = token
                    ip_flag = True
                    break
        except:
            ip_flag = False

    try:
        robot_id = int(self_ip.split('.')[-1])
    except:
        print("DEBUG: Could not parse robot ID, using 99")

    print("DEBUG: Robot IP:", self_ip, "ID:", robot_id)

    # Load JSON data
    try:
        with open('params.json', 'r') as f:
            json_data = json.load(f)
    except Exception as e:
        print("DEBUG: Error reading params.json:", e)
        return 'reload', voltage_status

    # Start logging
    csvfile, writer = logbook.open_file(robot_id)
    writer.writerow([now(), 'bootloader', 'Starting main...'])
    csvfile.flush()

    # Create shared_data
    shared_data = lib.shared_data_management.SharedData(robot_id)
    data_manager = lib.shared_data_management.SharedDataManager(shared_data)

    print("DEBUG: Skipping all base station comms. No ATC connection will be established.")

    try:
        # Minimal ProcessManager
        process_manager = ProcessManager(
            robot_id,
            self_ip,
            json_data['localizer_ip'],
            json_data['localizer_port'],
            json_data['localizer_timeout'],
            shared_data  # pass shared_data here
        )
        process_manager.set_user_code(json_data['user_code'])

        # Set up state machine
        stm = lib.bootloader_stm.BootloaderStateMachine(json_data['fc_watchdog'], json_data['server_ip'])

        # Start up
        startup_sequence(stm, process_manager, data_manager, writer, now())
        writer.writerow([now(), 'bootloader', 'Processes started, state machine idle.'])
        csvfile.flush()

        # Main run loop – no ATC
        while True:
            # Let the state machine handle checks
            if stm.current_state.id in ['idle', 'running']:
                abort = stm.check(process_manager, data_manager, writer, now())
                if abort:
                    writer.writerow([now(), 'bootloader', 'State machine triggered abort -> shutting down.'])
                    csvfile.flush()
                    try:
                        send_message_to_state_machine('shutdown', stm, process_manager, data_manager, writer, csvfile, now())
                    except:
                        pass
                    break

            time.sleep(0.05)

    except KeyboardInterrupt:
        writer.writerow([now(), 'bootloader', 'KeyboardInterrupt in main'])
        csvfile.flush()
    except Exception as e:
        writer.writerow([now(), 'bootloader', f'Error in main(): {str(e)}'])
        trace = traceback.extract_tb(e.__traceback__)
        for t in trace:
            writer.writerow([str(t)])
        writer.writerow([str(type(e).__name__), str(e)])
        csvfile.flush()
        ret_val = 'reload'
    finally:
        writer.writerow([now(), 'bootloader', f'Final shutdown: ret_val={ret_val}'])
        process_manager.kill_all()
        writer.writerow([now(), 'bootloader', 'All processes killed. End main.'])
        csvfile.flush()
        csvfile.close()

    return ret_val, voltage_status


def startup_sequence(stm, process_manager, data_manager, writer, t):
    '''
    Goes from initial state -> preflight -> idle.
    '''
    process_manager.startup()  # spawns processes
    writer.writerow([t(), 'bootloader', 'Creating processes, done.'])

    stm.create(process_manager, data_manager.shared_data)
    writer.writerow([t(), 'bootloader', 'Processes created. Doing preflight checks...'])
    stm.do_preflight_checks(process_manager)
    stm.checks_complete()
    writer.writerow([t(), 'bootloader', 'State machine is now idle.'])


def send_message_to_state_machine(message, stm, process_manager, data_manager, writer, csvfile, t):
    '''
    If you want to talk to the state machine at runtime. We just do "shutdown" as an example.
    '''
    if message == 'shutdown':
        writer.writerow([t(), 'bootloader', 'Shutting down the state machine.'])
        stm.shutdown(process_manager)
        time.sleep(1)
        return True
    return False


class ProcessManager:
    '''
    Minimal manager that creates local processes for:
      localizer, fc_handler, control_manager, user_code_handler, comms sender/receiver
    Passes the REAL shared_data to each process so they can do data_manager stuff properly.
    '''
    def __init__(self, robot_id, ip, localizer_ip, localizer_port, localizer_timeout, shared_data):
        self.robot_id = robot_id
        self.ip = ip
        self.localizer_ip = localizer_ip
        self.localizer_port = localizer_port
        self.localizer_timeout = localizer_timeout
        self.ucf = 'none'
        self.processes = []
        self.objects = {}
        self.pipes = {}
        self.shared_data = shared_data  # store the same shared_data object

    def set_user_code(self, user_code):
        self.ucf = user_code

    def startup(self):
        # create processes
        self.make_processes()
        # start all processes except user_code (optionally)
        for p in self.processes:
            if p.name != 'user code handler':
                p.start()

    def make_processes(self):
        # define which processes to spawn
        name_list = [
            'localizer',
            'fc handler',
            'control manager',
            'user code handler',
            'sender',
            'receiver'
        ]
        for name in name_list:
            parent_conn, child_conn = mp.Pipe()

            if name == 'localizer':
                obj = localize.LocalizerUDP(
                    self.robot_id, child_conn,
                    self.localizer_ip, self.localizer_port,
                    self.localizer_timeout
                )
                # pass self.shared_data to the run function
                proc = mp.Process(target=obj.run, args=(self.shared_data,), name=name)

            elif name == 'fc handler':
                obj = fc_handler.FlightControllerInterface(self.robot_id, child_conn)
                proc = mp.Process(target=obj.run, args=(self.shared_data,), name=name)

            elif name == 'control manager':
                obj = control_manager.ControlManager(self.robot_id, child_conn)
                proc = mp.Process(target=obj.run, args=(self.shared_data,), name=name)

            elif name == 'user code handler':
                obj = user_code_handler.UserCodeHandler(self.robot_id, child_conn, self.ucf)
                proc = mp.Process(target=obj.run, args=(self.shared_data,), name=name)
                self.user_code_process = proc

            elif name == 'sender':
                obj = comms.Sender(self.robot_id, child_conn, self.ip)
                proc = mp.Process(target=obj.run, args=(self.shared_data,), name=name)

            elif name == 'receiver':
                obj = comms.Receiver(self.robot_id, child_conn, self.ip)
                proc = mp.Process(target=obj.run, args=(self.shared_data,), name=name)
            else:
                continue

            self.objects[name] = obj
            self.pipes[name] = parent_conn
            self.processes.append(proc)

    def kill_all(self):
        # Let each object do its .exit() routine
        for obj in self.objects.values():
            try:
                obj.exit()
            except:
                pass

        # Terminate each process
        for p in self.processes:
            if p.is_alive():
                p.terminate()
                p.join()

        self.processes.clear()
        self.objects.clear()
        self.pipes.clear()

    def kick_off_user_code(self):
        # if you want to start user code separately
        if hasattr(self, 'user_code_process'):
            self.user_code_process.start()
            return True
        return False

    def reset_user_code(self):
        pass  # stub if needed for advanced logic


if __name__ == "__main__":
    voltage_status = 'normal'
    tries = 0
    while tries < MAX_TRYS:
        ret_val, voltage_status = main(voltage_status)
        print(f"DEBUG: main() returned {ret_val}, {voltage_status}")
        if ret_val == 'normal':
            break
        else:
            tries += 1
            print("DEBUG: Retrying main() in 2s")
            time.sleep(2)
            # Optionally reload modules
            try:
                importlib.reload(localize)
                importlib.reload(fc_handler)
                importlib.reload(control_manager)
                importlib.reload(user_code_handler)
                importlib.reload(logbook)
                importlib.reload(comms)
                importlib.reload(lib.shared_data_management)
                importlib.reload(lib.bootloader_stm)
            except Exception as e:
                print(f"DEBUG: Error reloading modules: {e}")
                break

    print("DEBUG: Exiting bootloader after tries=", tries)
