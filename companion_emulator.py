"""
Samuel Dudley - December 2015
SITL emulator for simple nav payload

Adapted November 2017 to support "SLAM like" output
for EKF3 development
"""
import time, threading, select, sys, os

# setup up mavlink 
os.environ['MAVLINK20'] = '1' # force MAVLink v2 for the moment
from pymavlink import mavutil
# now mavlink has been setup we can inport dependant modules
import companion_est, companion_common 

class Companion_FC(object):
    def __init__(self, optsargs):
        (self.opts, self.args) = optsargs
        try:
            self.connection = companion_common.Connection(mavutil.mavlink_connection(self.opts.connection))
        except Exception as err:
            print("Failed to connect to %s : %s" %(self.opts.connection,err))
            sys.exit(1)
        
        self.est = companion_est.Companion_EST(optsargs, self.connection)
        self.state = {'EST':self.est.state}
        
        self.debug = self.opts.debug

        if self.opts.recv_block:
            # block waiting for a particular msg type
            print('Waiting for', self.opts.recv_block)
            msg = self.connection.control_connection.recv_match(type = self.opts.recv_block, blocking=True)
            
            # print what we got
            print 'ID:', msg.get_msgId()
            print 'TYPE:', msg.get_type()
            print 'SYSTEM:', msg.get_srcSystem()
            print 'COMPONENT:', msg.get_srcComponent()
            print msg.to_dict()

            self.shutdown() # kill the thread
    
    def check_triggers(self,  one_shot = False):
        state = self.state
        
        for periodic_event in state['EST'].periodic_events:
            if state['EST'].periodic_events[periodic_event].trigger(clock = state['EST'].ap_sec_since_boot()):
                
                if periodic_event == 'HEARTBEAT':
                    if (self.opts.heartbeat):
                        if self.opts.debug:
                            print periodic_event
                        self.est.send_msg(periodic_event)
                        
                else:
                    if self.opts.debug:
                        print periodic_event
                    self.est.send_msg(periodic_event)
    
    def process_connection_in(self):
        state = self.state
        inputready,outputready,exceptready = select.select([self.connection.control_connection.port],[],[])#,0) # timeout  = 0 will cause non-blocking behaviour
        for s in inputready:

            msg = self.connection.control_connection.recv_msg()
        
                
            if msg != None:
                msg_id = msg.get_msgId()
                msg_type = msg.get_type()
                msg_dict = msg.to_dict()
                msg_sys = msg.get_srcSystem()
                msg_comp = msg.get_srcComponent()
                
#                 print msg_type
                
                if msg_type == 'HEARTBEAT':
                    # Heartbeat from the AP (and the gcs if routed?)
                    if self.debug:
                        print 'got', msg_type
                    for x in state:
                        state[x].update_ap_heartbeat()
                
                if msg_type == 'CAMERA_FEEDBACK_AHRS':
                    # Camera feedback event detected
                    # the estimator will 'process' the image
                    if self.debug:
                        print 'got', msg_type
                    
                    self.est.update_est(msg_dict)
                    
                if msg_type in ['SIMSTATE', 'GPS_RAW_INT', 'GLOBAL_POSITION_INT', 'ATTITUDE', 'LOCAL_POSITION_NED']:
                    # NOTE: These messages are hijacked to populate the true_est state
                    if self.debug:
                        print 'got', msg_type
                        
                    if msg_type == 'GPS_RAW_INT':
                        # update the clock
                        for state in self.state.values():
                            ap_last = state.clock['ap']
                            ap_current = msg_dict['time_usec']*10.**-3
                            wall_last = state.clock['wall']
                            wall_current = state.millisec_since_boot()
                            ap_delta = ap_current - ap_last
                            wall_delta = wall_current - wall_last
                            clock_ratio = ap_delta/wall_delta
                            state.clock['wall'] = wall_current
                            state.clock['ap'] = ap_current
                            state.clock['ratio'] = clock_ratio
                            
                            if state.clock['ap_start'] is not None:
                                state.clock['ave_ratio'] = (ap_current - state.clock['ap_start'])/(wall_current - state.clock['wall_start'])
                            
                            if state.clock['ap_start'] is None:
                                state.clock['ap_start'] = ap_current
                                state.clock['wall_start'] = wall_current
                            
                    self.est.update_true(msg)
                    
    def is_alive(self):
        state = self.state
        alive_list = [state[x].is_alive() for x in state]
        if False not in alive_list:
            return True
        else:
            return False
    
    def shutdown(self):
        state = self.state
        for x in state:
            state[x].shutdown()
    
def main_loop(optargs):
    '''main processing loop'''
    flight_computer = Companion_FC(optargs)
    while flight_computer.is_alive():
        flight_computer.process_connection_in()
        flight_computer.check_triggers()
#         time.sleep(0.05) # not needed when using blocking select calls

if __name__ == '__main__':
    from optparse import OptionParser
    
    parser = OptionParser('companion_emulator.py [options]')
    
    parser.add_option("-c", "--connection", dest="connection", type='str',
                      help="companion connection", default="tcp:127.0.0.1:5763")
    
    parser.add_option("-d", "--debug", dest="debug",
                      help="Enable debug output", default=False, action="store_true")
    
    parser.add_option("-n", "--is-ned", dest="is_ned",
                      help="Frame is assumed to be NED if true", default=False, action="store_true")
    
    parser.add_option("-s", "--scale-unknown", dest="scale_unknown",
                      help="Frame scaling is assumed unknown or not in metres", default=False, action="store_true")
    
    parser.add_option("-b", "--heartbeat", dest="heartbeat",
                      help="Send heartbeats from companion computer payload", default=False, action="store_true")
    
    parser.add_option("--random-scale", dest="random_scale",
                      help="set the word scale to be a randomly selected value", default=False, action="store_true")
    
    parser.add_option("--random-quat", dest="random_quat",
                      help="set the word rotation to be a randomly generated quaternion", default=False, action="store_true")
    
    parser.add_option("-r", "--recv-block", dest="recv_block",
                      help="Wait for a msg type from the AP and exit", default=False, choices=[False, 'HEARTBEAT',
                                                                                               'SIMSTATE', 'CAMERA_FEEDBACK_AHRS'])
    
    # run main loop as a thread
    main = threading.Thread(target = main_loop, args=(parser.parse_args(),))
    main.daemon = True
    main.start()
    main.join()
    

"""
navpy is required: https://github.com/NavPy/NavPy
"""
