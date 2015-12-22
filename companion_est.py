from pymavlink import mavutil
import companion_common
from util import transformations as tr
import math, random
import numpy as np
import os
import navpy # download from github (https://github.com/NavPy/NavPy) and add to your PYTHONPATH

class ESTState(companion_common.State):
    """holds the companion estimator state"""
    def __init__(self, optsargs, connection):
        super(ESTState, self).__init__(optsargs)
        (opts, args) = optsargs
        
        execfile(os.path.join(os.getcwd(), 'companion_errors.py'))
        
        self.est_state_last_update = 0
        self.est_state = {}
        self.est_home = {}
        self.est_reported = {}
        self.certainty = {'hacc':0, 'vacc':0}
        self.status = {'last_reset_time':0, 'reset_counter':0}
        self.feedback_msg = None
        
        self.component = 191
        self.system = 1
        self.connection = connection
        self.periodic_events = {}
        self.periodic_events['HEARTBEAT'] = companion_common.periodic_event(1 , clock = self.ap_sec_since_boot()) # send a heartbeat msg at 1Hz
        self.periodic_events['DATASTREAM'] = companion_common.periodic_event(1 , clock = self.ap_sec_since_boot())
        # !!! BOOTSTRAP time is set in companion_errors.py NOT HERE !!!
        self.periodic_events['BOOTSTRAP'] = companion_common.periodic_event(1., clock = self.ap_sec_since_boot()) # this timer is only to delay init the estimator state
        # !!! BOOTSTRAP time is set in companion_errors.py NOT HERE !!!
        
        self.true_state = {'lat':0,
                           'lon':0,
                           'alt_wgs84':0,
                           'alt_agl':0,
                           'roll':0,
                           'pitch':0,
                           'yaw':0,
                           'gspeed':0,
                           'gcourse':0,
                           'vspeed':0}
        
        
        # Setup the world scale
        self.status['scale_unknown'] = opts.scale_unknown
        if self.status['scale_unknown']:
            if opts.random_scale:
                # make a random scale factor from the range [0.1, 100]
                self.true_state['scale_factor'] = random.uniform(0.1, 100)
            else:
                # or set it value defined in the external file
                self.true_state['scale_factor'] = self.errors['ext_scale_factor']
        else:
            self.true_state['scale_factor'] = 1.0
        print 'World scale factor: ', self.true_state['scale_factor']
        
        # Setup the world rotation
        self.status['is_ned'] = opts.is_ned
        if self.status['is_ned']:
            # use a null rotation if NED
            self.rot_offset = [1,0,0,0]
        else:
            if opts.random_quat:
                # make a random quaternion
                self.rot_offset = tr.random_quaternion()
            else:
                # world rotation defined in the external file
                self.rot_offset = tr.quaternion_from_euler(np.radians(self.errors['ext_rot_offset']['yaw']), # yaw
                                                           np.radians(self.errors['ext_rot_offset']['pitch']), # pitch
                                                           np.radians(self.errors['ext_rot_offset']['roll']), # roll
                                                           'rzyx')
        print 'World rotation quaternion: ', self.rot_offset 

    
        
class Companion_EST(object):
    def __init__(self, optsargs, connection):
        self.state = ESTState(optsargs, connection)
        self.send_msg('HEARTBEAT') # send one just to let the AP know we are here... 
    
    def bootstrap(self, msg_dict, reason):
        '''reset the est to ap supplied values'''
        state = self.state
        if reason in state.errors['bootstrap'].keys():
            response = state.errors['bootstrap'][reason]

        elif reason == 'init':
            if state.true_state['alt_wgs84'] < state.errors['bootstrap']['init_alt_wgs84']:
                response = False
                reason = 'waiting for alt_wgs84 >= {0}, current alt_wgs84 = {1}'.format(state.errors['bootstrap']['init_alt_wgs84'], state.true_state['alt_wgs84'])
            else:
                response = True
                # store the initial state as the home point (used to generate the exteral sensor coord system)
                state.est_home['lat'] = msg_dict['lat']*10.**-7
                state.est_home['lon'] = msg_dict['lon']*10.**-7
                state.est_home['alt_wgs84'] = msg_dict['alt_wgs84']*10.**-3
        else:
            response = False
        
        if response:
            for key in state.true_state:
                state.est_state[key] = state.true_state[key]
            state.est_state['lat'] = msg_dict['lat']*10.**-7
            state.est_state['lon'] = msg_dict['lon']*10.**-7
            state.est_state['alt_wgs84'] = msg_dict['alt_wgs84']*10.**-3

            for key in state.errors['walk_val']:
                state.errors['walk_val'][key] = 0
                
            print 'Bootstrap',response, reason
            if state.feedback_msg:
                state.status['last_reset_time'] = int(state.feedback_msg['trigger_time'])
            else:
                state.status['last_reset_time'] = 0
                
            state.status['reset_counter'] += 1
            return True
        else:
            print 'Bootstrap',response, reason
            return False
        
    def update_est(self, msg_dict):
        '''update the est from the true state'''
        # called when a CAMERA_FEEDBACK_AHRS msg is received by the CC
        
        state = self.state
        state.feedback_msg = msg_dict # keep the feedback message values in the state
#         print 'ahrs quat:', (msg_dict['q1'], msg_dict['q2'],msg_dict['q3'], msg_dict['q4'])
#         print 'euler_from_quaternion ahrs:', tr.euler_from_quaternion([msg_dict['q1'], msg_dict['q2'],msg_dict['q3'],msg_dict['q4']])
        
        if None in state.clock.values():
            print 'clock None'
            return False # bail out here
        
        # work out how many seconds have passed in AP time since the last estimate was sent
        est_delta_sec = (state.millisec_since_boot() - state.est_state_last_update)* state.clock['ave_ratio'] *10.**-3
            
        if state.est_state == {} or None in state.est_state.values():
            return False # bail out here
        

        est_noise = state.errors['noise'] # sensor noise
        est_bias = state.errors['bias'] # sensor bias
        est_lower = state.errors['lower'] # lower limit for sensor value
        est_upper = state.errors['upper'] # upper limit for sensor value
        
        est_walk_val = state.errors['walk_val'] # contains the current walk value (gets reset to 0 with a bootstrap)
        est_walk_bias = state.errors['walk_bias'] # if the bias is zero the walk will be random
        est_walk_noise = state.errors['walk_noise'] # the size of the possible walk step
        est_walk_lower = state.errors['walk_lower'] # walk value lower limit
        est_walk_upper = state.errors['walk_upper'] # walk value upper limit
        
        # calculate the new walk values
        # new walk = noise(old walk + walk bias, walk noise)
        # new walk = limit(new walk)

        for key in state.est_state:
            est_walk_val[key] = companion_common.noise(est_walk_val[key] + (est_walk_bias[key]*est_delta_sec), est_walk_noise[key])
            est_walk_val[key] = companion_common.limit(est_walk_val[key], est_walk_lower[key], est_walk_upper[key])
        
        # calculate the new estimate values
        for key in state.est_state:
            if key in ['roll', 'pitch', 'yaw']:
                state.est_state[key] = companion_common.wrap_180(
                                                             companion_common.noise(state.true_state[key]+est_bias[key]+est_walk_val[key],
                                                                                est_noise[key]
                                                                                )
                                                             )
                    
            elif key in ['gcourse']:
                state.est_state[key] = companion_common.wrap_360(
                                                             companion_common.noise(state.true_state[key]+est_bias[key]+est_walk_val[key],
                                                                                est_noise[key]
                                                                                )
                                                             )
                
            
                
            elif key in ['gspeed', 'alt_wgs84', 'alt_agl', 'vspeed', 'scale_factor']:
                   state.est_state[key] = companion_common.limit(
                                                             companion_common.noise(state.true_state[key]+est_bias[key]+est_walk_val[key],
                                                                                est_noise[key]
                                                                                ),
                                                             lower = est_lower[key] , upper = est_upper[key]
                                                             )
            
            else:
                # this value needs to be handled in a unique way e.g. lat, lon
                pass

        (state.est_state['lat'], state.est_state['lon']) = companion_common.gps_newpos(
                                                                                   state.est_state['lat'], state.est_state['lon'], state.est_state['gcourse'],
                                                                                   state.est_state['gspeed']*est_delta_sec
                                                                                   )
        
        
        # apply the noise on the estimated state to give the reported pos from the estimator...
        state.est_reported['lat'] = companion_common.noise(state.est_state['lat']+est_bias['lat'], est_noise['lat'])
        state.est_reported['lon'] = companion_common.noise(state.est_state['lon']+est_bias['lon'], est_noise['lon'])
        
        state.est_state_last_update = state.millisec_since_boot()
        
        if not self.update_distance():
            return False
        
        if not self.update_certainty():
            return False
        
        (state.est_reported['north'],
         state.est_reported['east'],
         state.est_reported['down']) = navpy.lla2ned(state.est_reported['lat'], state.est_reported['lon'], state.est_state['alt_wgs84'],
                                            state.est_home['lat'], state.est_home['lon'], state.est_home['alt_wgs84'])
        
        [state.est_reported['q1'],
         state.est_reported['q2'],
         state.est_reported['q3'],
         state.est_reported['q4']] = tr.quaternion_from_euler(np.radians(state.est_state['yaw']),
                                                              np.radians(state.est_state['pitch']),
                                                              np.radians(state.est_state['roll']),
                                                              'rzyx')

        print 'image id  :', state.feedback_msg['img_idx']
        print 'pre r ahrs:', np.degrees(tr.euler_from_quaternion([state.est_reported['q1'],
                                                                 state.est_reported['q2'],
                                                                 state.est_reported['q3'],
                                                                 state.est_reported['q4']]))
        print 'pre scale :',(state.est_reported['north'], state.est_reported['east'], state.est_reported['down'])

        self.apply_scale_factor()
        self.apply_frame_rotation()
        self.send_msg('EXT_NAV') # send the position estimate back to the AP
        
        return True
        
    def apply_scale_factor(self):
        state = self.state
        scale_factor = state.est_state['scale_factor']
        state.est_reported['north'] *= scale_factor
        state.est_reported['east'] *= scale_factor
        state.est_reported['down'] *= scale_factor
    
    def apply_frame_rotation(self):
        state = self.state
        
        # rotate the location
        [state.est_reported['north'],
         state.est_reported['east'],
         state.est_reported['down']] = tr.quaternion_transform(state.rot_offset, [state.est_reported['north'], state.est_reported['east'], state.est_reported['down']])

        # rotate the attitude
        [state.est_reported['q1'],
         state.est_reported['q2'],
         state.est_reported['q3'],
         state.est_reported['q4']] =  tr.quaternion_multiply(tr.quaternion_inverse(state.rot_offset),
                                                             [state.est_reported['q1'],
                                                              state.est_reported['q2'],
                                                              state.est_reported['q3'],
                                                              state.est_reported['q4']])
        print 'final ahrs:', np.degrees(tr.euler_from_quaternion([state.est_reported['q1'],
                                                         state.est_reported['q2'],
                                                         state.est_reported['q3'],
                                                         state.est_reported['q4']]))
         
        print 'final pos :', (state.est_reported['north'], state.est_reported['east'], state.est_reported['down'])
        print ""
                                                    
    def update_certainty(self):
        # TODO update rotational accuracy
        state = self.state
        if state.est_state == {}:
            return False
        
        state.certainty['hacc'] = state.errors['distance']['h_distance']
        state.certainty['vacc'] = state.errors['distance']['v_distance']
        
        
        h_distance = state.errors['distance']['h_distance']
        v_distance = state.errors['distance']['v_distance']
        
        pos_error =  math.sqrt(h_distance**2 + v_distance**2)
        if pos_error < 0.01:
            state.certainty['posacc'] = 0.01
        else:
            state.certainty['posacc'] = pos_error
        
        
        quat_true = tr.quaternion_from_euler(np.radians(state.true_state['yaw']),
                                            np.radians(state.true_state['pitch']),
                                            np.radians(state.true_state['roll']),
                                            'rzyx')
        
        quat_est = tr.quaternion_from_euler(np.radians(state.est_state['yaw']),
                                            np.radians(state.est_state['pitch']),
                                            np.radians(state.est_state['roll']),
                                            'rzyx')
        
        # TODO FIXME !!!
        state.certainty['rotacc'] = 0.1#2.0 * np.arccos(np.dot(quat_true, quat_est))
        return True
        
        
    def update_distance(self, allow_loopback = True):
        '''update the distance between the true and estimated state'''
        state = self.state
        if state.est_state == {}:
            return False
        
        h_distance = companion_common.get_h_distance(state.true_state['lat'], state.true_state['lon'],
                                                         state.est_state['lat'],state.est_state['lon'])
        
        v_distance = companion_common.get_v_distance(state.true_state['alt_wgs84'], state.est_state['alt_wgs84'])
        
        state.errors['distance']['h_distance'] = h_distance
        state.errors['distance']['v_distance'] = v_distance
        
        distance = math.sqrt(h_distance**2 + v_distance**2)
        
        bootstrap_distance = state.errors['bootstrap']['distance']
        if (allow_loopback and bootstrap_distance and distance > bootstrap_distance):
            self.bootstrap({'lat':state.true_state['lat']*10.**7, 'lon':state.true_state['lon']*10.**7, 'alt_wgs84':state.true_state['alt_wgs84']*10.**3},
                           reason = 'distance')
            self.update_distance(allow_loopback = False)
        return True
        
    def update_true(self, m):
        '''update the true state from the SITL'''
        # ['SIMSTATE', 'GPS_RAW_INT', 'GLOBAL_POSITION_INT']
        state = self.state
        msg_dict = m.to_dict()
        msg_type = m.get_type()

        if msg_type == 'GPS_RAW_INT':
            state.true_state['gcourse'] = companion_common.wrap_360(msg_dict['cog']*10.**-2)
            # eph and epv

        if msg_type == 'GLOBAL_POSITION_INT':
            state.true_state['alt_wgs84'] = msg_dict['alt']*10.**-3
            state.true_state['alt_agl'] = msg_dict['relative_alt']*10.**-3
            state.true_state['gspeed'] = math.sqrt((abs(msg_dict['vx']*10.**-2))**2 + (abs(msg_dict['vy']*10.**-2))**2)
            state.true_state['vspeed'] = msg_dict['vz']*10.**-2
            
        if msg_type == 'SIMSTATE':
            state.true_state['roll'] = np.degrees(msg_dict['roll'])
            state.true_state['pitch'] = np.degrees(msg_dict['pitch'])
            state.true_state['yaw'] = companion_common.wrap_180(np.degrees(msg_dict['yaw'])) # converts from [-180, +180] to [0,360]
            state.true_state['lat'] = msg_dict['lat']*10.**-7
            state.true_state['lon'] = msg_dict['lng']*10.**-7
            
#             print 'euler fdm:', (msg_dict['yaw'],
#                              msg_dict['pitch'],
#                              msg_dict['roll'])
#             print 'quaternion_from_euler fdm:', tr.quaternion_from_euler(msg_dict['yaw'],
#                                            msg_dict['pitch'],
#                                            msg_dict['roll'],
#                                            'rzyx')
            
    def send_msg(self, msg_type):
        state = self.state
        state.connection.set_component(state.component)
        state.connection.set_system(state.system)
        
        m = None
        
        if msg_type == 'HEARTBEAT':
             
            # MAV_TYPE_ONBOARD_CONTROLLER = 18 # Onboard companion controller
            # MAV_AUTOPILOT_INVALID = 8 # No valid autopilot, e.g. a GCS or other MAVLink component
            m = state.connection.control_link.heartbeat_encode(18, 8, base_mode = 1, custom_mode = 0, system_status = 4)
        
        elif msg_type == 'DATASTREAM':
            rate_hz = 50
            start_stop = 1 # start == 1 stop ==0
            m = state.connection.control_link.request_data_stream_encode(1, 1,
                                                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                                rate_hz, start_stop)
                
        elif msg_type == 'BOOTSTRAP':
            bootstrap_time = state.errors['bootstrap']['time']
            if state.status['reset_counter'] == 0:
                ret = self.bootstrap({'lat':state.true_state['lat']*10.**7, 'lon':state.true_state['lon']*10.**7, 'alt_wgs84':state.true_state['alt_wgs84']*10.**3},
                               reason = 'init')
                if ret: # the inital bootstrap was OK
                    try:
                        bootstrap_frequency = 1./bootstrap_time
                    except:
                        bootstrap_frequency = 0
                        
                    state.periodic_events['BOOTSTRAP'].frequency = bootstrap_frequency
                else: # the inital bootstrap failed for some reason
                    # keep trying to bootstrap at 1Hz for now...
                    pass
                    
            elif bootstrap_time:
                self.bootstrap({'lat':state.true_state['lat']*10.**7, 'lon':state.true_state['lon']*10.**7, 'alt':state.true_state['alt_wgs84']*10.**3},
                               reason = 'time')
                try:
                    bootstrap_frequency = 1./bootstrap_time
                except:
                    bootstrap_frequency = 0
                    
                state.periodic_events['BOOTSTRAP'].frequency = bootstrap_frequency
                
            else: 
                # bootstrap_time is false
                pass

        
        elif msg_type == 'EXT_NAV':
            m = state.connection.control_link.ext_nav_encode(target_system = 1, # the AP
                                                              target_component = 1, # the AP
                                                              time_usec = state.millisec_since_boot(), # not correct, should be micro
                                                              time_measurement_msec = int(state.feedback_msg['trigger_time']), # value taken from CAMERA_FEEDBACK_AHRS !must be millis!
                                                              time_reset_msec = state.status['last_reset_time'], # to be updated by bootstrap or 'loop closure' event
                                                              pos_x = state.est_reported['north'],
                                                              pos_y = state.est_reported['east'],
                                                              pos_z = state.est_reported['down'],
                                                              quat_q1 = state.est_reported['q1'],
                                                              quat_q2 = state.est_reported['q2'],
                                                              quat_q3 = state.est_reported['q3'],
                                                              quat_q4 = state.est_reported['q4'],
                                                              pos_error = state.certainty['posacc'], # TODO what to do if not scaled in m? Dont send zeros!
                                                              ang_error = state.certainty['rotacc'], # FIXME NaNs are causing AP to crash, currently set to 0.1 ~6 degrees
                                                              offset_x = 0, # TODO
                                                              offset_y = 0, # TODO
                                                              offset_z = 0, # TODO
                                                              scale_unknown = int(state.status['scale_unknown']), # 1 or 0
                                                              frame_is_NED = int(state.status['is_ned']) # 1 or 0
                                                              )
        
#         <message id="11021" name="EXT_NAV">
#             <description>local frame external position and attitude estimate</description>
#             <field type="uint8_t" name="target_system">System ID</field>
#             <field type="uint8_t" name="target_component">Component ID</field>
#             <field type="uint64_t" name="time_usec">Timestamp (micros since boot or Unix epoch)</field>
#             <field type="uint32_t" name="time_measurement_msec">Autopilot system time that the measurement for this estimate was taken (mSec)</field>
#             <field type="uint32_t" name="time_reset_msec">Autopilot system time when the last estimate reset was requested (mSec)</field>
#             <field type="float" name="pos_x">X position in the RH navigation frame. Frame is assumed to be NED if frame_is_NED is 1. Units are meters if scale_unknown is 0 </field>
#             <field type="float" name="pos_y">Y position in the RH navigation frame. Frame is assumed to be NED if frame_is_NED is 1. Units are meters if scale_unknown is 0 </field>
#             <field type="float" name="pos_z">Z position in the RH navigation frame. Frame is assumed to be NED if frame_is_NED is 1. Units are meters if scale_unknown is 0 </field>
#             <field type="float" name="quat_q1">Quaternion component desribing the the rotation from navigation frame to body frame </field>
#             <field type="float" name="quat_q2">Quaternion component desribing the the rotation from navigation frame to body frame </field>
#             <field type="float" name="quat_q3">Quaternion component desribing the the rotation from navigation frame to body frame </field>
#             <field type="float" name="quat_q4">Quaternion component desribing the the rotation from navigation frame to body frame </field>
#             <field type="float" name="pos_error">1-sigma spherical position error. Units are meters if scale_unknown is 0</field>
#             <field type="float" name="ang_error">1-sigma spherical angle error (rad)</field>
#             <field type="float" name="offset_x">X position offset of the external navigation sensor in body frame (m)</field>
#             <field type="float" name="offset_y">Y position offset of the external navigation sensor in body frame (m)</field>
#             <field type="float" name="offset_z">Z position offset of the external navigation sensor in body frame (m)</field>
#             <field type="uint8_t" name="scale_unknown">Set to 1 when the position scaling is unknown or not in metres, otherwise 0</field>
#             <field type="uint8_t" name="frame_is_NED">Set to 1 when if the external mavigaton system is using a NED coordinate frame, otherwise 0</field>
#         </message>
                    
        else:
            # unhandled msg
            pass
            
        if m is not None:
            state.connection.control_link.send(m)
            
        
         