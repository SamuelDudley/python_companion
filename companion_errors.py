

self.errors = {# scale factor 
               'ext_scale_factor':1.0,
               
               # frame offsets from ned
               'ext_rot_offset': {'roll':0, # degrees
                                  'pitch':0, # degrees
                                  'yaw':0}, # degrees
               
               
               # when do we want to bootstrap?
               'bootstrap':{'init_alt_wgs84':-1, # [meters] what altitude should we do the initial bootstrap
                            'time':False, # [seconds] (in sim time) between bootstrap events or False
                            'distance':100.0, # [meters] boot strap if the estimated soln is greater than this dist away from GPS pos
                            'msg':True}, # boot strap when we get a RESET_EST msg
               
               # sensor offsets (constant)
               'bias':{'lat':0,
                       'lon':0,
                       'alt_wgs84':0,
                       'alt_agl':0,
                       'roll':0,
                       'pitch':0,
                       'yaw':0,
                       'gspeed':0,
                       'gcourse':0,
                       'vspeed':0,
                       'scale_factor':0},
               
               # sensor noise
               'noise':{'lat':0.00000, #0.00005
                   'lon':0.00000, #0.00005
                   'alt_wgs84':0,
                   'alt_agl':0,
                   'roll':0,
                   'pitch':0,
                   'yaw':0,
                   'gspeed':0,
                   'gcourse':0,
                   'vspeed':0,
                   'scale_factor':0},
               
               # lower limit to reportable sensor value
               'lower':{'lat':None,
                   'lon':None,
                   'alt_wgs84':0,
                   'alt_agl':0,
                   'roll':None,
                   'pitch':None,
                   'yaw':None,
                   'gspeed':0,
                   'gcourse':None,
                   'vspeed':None,
                   'scale_factor':0},
               
               # upper limit to reportable sensor value
               'upper':{'lat':None,
                   'lon':None,
                   'alt_wgs84':None,
                   'alt_agl':0,
                   'roll':None,
                   'pitch':None,
                   'yaw':None,
                   'gspeed':None,
                   'gcourse':None,
                   'vspeed':None,
                   'scale_factor':None},
               
               # these values update over time during the sim and are reset with a boot strap
               'walk_val':{'lat':0,
                   'lon':0,
                   'alt_wgs84':0,
                   'alt_agl':0,
                   'roll':0,
                   'pitch':0,
                   'yaw':0,
                   'gspeed':0,
                   'gcourse':0,
                   'vspeed':0,
                   'scale_factor':0},
           
               # if the bias is 0 the walk will be random
               # bias is applied on a time basis. e.g. if gcourse = 10 we expect a 10 degree/s rotation to the right
               'walk_bias':{'lat':0,
                   'lon':0,
                   'alt_wgs84':0,
                   'alt_agl':0,
                   'roll':0,
                   'pitch':0,
                   'yaw':0,
                   'gspeed':0,
                   'gcourse':-0.0,#0.1, -0.03
                   'vspeed':0,
                   'scale_factor':0},
               
               # the standard deviation of the walk value
               'walk_noise':{'lat':0.000000, #0.0000005
                   'lon':0.000000, #0.0000005
                   'alt_wgs84':.0, #0.05
                   'alt_agl':.0, #0.05
                   'roll':0,
                   'pitch':0,
                   'yaw':0,
                   'gspeed':0.0, #0.01
                   'gcourse':0.0, #0.1
                   'vspeed':0.0, #0.01
                   'scale_factor': 0.0}, # could add a small amount of noise here
               
               # lower limit of the walk value
               'walk_lower':{'lat':None,
                   'lon':None,
                   'alt_wgs84':None,
                   'alt_agl':None,
                   'roll':None,
                   'pitch':None,
                   'yaw':None,
                   'gspeed':None,
                   'gcourse':None,
                   'vspeed':None,
                   'scale_factor':None},
               
               # upper limit of the walk value
               'walk_upper':{'lat':None,
                   'lon':None,
                   'alt_wgs84':None,
                   'alt_agl':None,
                   'roll':None,
                   'pitch':None,
                   'yaw':None,
                   'gspeed':None,
                   'gcourse':None,
                   'vspeed':None,
                   'scale_factor':None},
           
                'distance':{'h_distance':0,
                            'v_distance':0}
               }