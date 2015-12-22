from pymavlink import mavutil
import time
import datetime
import numpy as np
import math

class State(object):
    def __init__(self, optsargs):
        (self.opts, self.args) = optsargs
        self.periodic_events = {}
        
        self.last_ap_heartbeat_time = time.time()
        self.time_since_last_ap_heartbeat = 0
        
        self.boot_time = datetime.datetime.now()
        
        self.clock = {'ap':0, 'wall':0, 'ratio':0, 'ap_start':None, 'wall_start':None, 'ave_ratio':None}
        
        self.alive = True
    
    def is_alive(self):
        return self.alive
    
    def shutdown(self):
        self.alive = False
        
    def time_since_ap_heartbeat(self):
        return time.time() - self.last_ap_heartbeat_time
    
    def update_ap_heartbeat(self):
        self.last_ap_heartbeat_time = time.time()
        
    def millisec_since_boot(self):
        # TODO : look at AP time and if it wraps then reset self.boot_time
        current_time = datetime.datetime.now()
        time_delta = current_time - self.boot_time
        sec = float(time_delta.seconds)
        micro = float(time_delta.microseconds)
        milli = micro*0.001 + sec*1000.0
        return milli
    
    def ap_sec_since_boot(self):
        if self.clock['ave_ratio'] is not None:
            return self.millisec_since_boot()* self.clock['ave_ratio'] *10.**-3
        else:
            return 0
    
class Connection(object):
    def __init__(self, connection):
        self.control_connection = connection # a MAVLink connection
        self.control_link = mavutil.mavlink.MAVLink(self.control_connection)
        self.control_link.srcSystem = 1
        self.control_link.srcComponent = 191
        
    def set_component(self, val):
        self.control_link.srcComponent = val
    
    def set_system(self, val):
        self.control_link.srcSystem = val

class periodic_event(object):
    '''a class for fixed frequency events'''
    def __init__(self, frequency, clock = None):
        if clock is None:
            self.wall_time = True
        else:
            self.wall_time = False
        
        self.frequency = float(frequency)
        if self.wall_time:
            self.last_time = time.time()
        else:
            self.last_time = clock

    def force(self):
        '''force immediate triggering'''
        self.last_time = 0
        
    def trigger(self, clock = None):
        '''return True if we should trigger now'''
        if self.wall_time:
            tnow = time.time()
        else:
            if clock is not None:
                tnow = clock
            else:
                print 'Error: Clock init was not wall time, supply clock value with .trigger(clock=)'
                sys.exit(1)
        if self.frequency == 0:
            return False
        
        if tnow < self.last_time:
            if self.wall_time:
                print("Warning, time moved backwards. Restarting timer.")
            self.last_time = tnow

        if self.last_time + (1.0/self.frequency) <= tnow:
            self.last_time = tnow
            return True
        return False
        
def noise(val, amp):
    if amp > 0:
        return np.random.normal(loc=val, scale=amp)
    else: #do nothing
        return val

def wrap_180(angle):
    if angle > 180:
        angle = angle % 360.0 - 360.0
    if angle < -180:
        angle = angle % 360.0 + 360.0
    
    if angle > 180:
        angle -= 360.0
    if angle < -180:
        angle += 360.0
    return angle

def wrap_360(angle):
    if angle > 360 or angle < 0:
        angle = angle % 360.0
    return angle

def limit(value, lower = None, upper = None):
    if lower is not None and value < lower:
        value = lower
    if upper is not None and value > upper:
        value = upper
    return value

def roll_limit(value, lower, upper):
    if value < lower:
        value = upper - abs(value-lower)
    if value > upper:
        value = lower + abs(value-upper)
    return value

def gps_range_bearing( lat1, lon1, lat2, lon2):
    distance = get_h_distance(lat1, lon1, lat2, lon2)

    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    dLat = lat1 - lat2
    dLon = lon1 - lon2
    rad_lat = wgs84_radius(lat1)
#    print rad_lat
#    print radius_of_earth
    dLat = dLat*rad_lat
    dLon = dLon*radius_of_earth


    bearing = math.atan2(dLon, dLat)
    
    return (distance,math.degrees(bearing))

def get_h_distance(lat1, lon1, lat2, lon2):
    '''get the horizontal distance between estimate and vehicle'''
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    dLat = lat2 - lat1
    dLon = lon2 - lon1
    
    # math as per mavextra.distance_two()
    a = math.sin(0.5 * dLat)**2 + math.sin(0.5 * dLon)**2 * math.cos(lat1) * math.cos(lat2)
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return 6371. * 1000. * c

def get_v_distance(alt1, alt2):
    '''get the horizontal distance between estimate and vehicle'''
    return abs(alt2 - alt1)

# Earth radius at a given latitude, according to the WGS-84 ellipsoid [m]
def wgs84_radius(lat,a = 6378137.0 ,b = 6356752.3142):
    # http://en.wikipedia.org/wiki/Earth_radius
    An = a*a * math.cos(lat)
    Bn = b*b * math.sin(lat)
    Ad = a * math.cos(lat)
    Bd = b * math.sin(lat)
    return math.sqrt( (An*An + Bn*Bn)/(Ad*Ad + Bd*Bd) )

radius_of_earth = 6378137.0 # in meters

def wrap_valid_longitude(lon):
    ''' wrap a longitude value around to always have a value in the range
        [-180, +180) i.e 0 => 0, 1 => 1, -1 => -1, 181 => -179, -181 => 179
    '''
    return (((lon + 180.0) % 360.0) - 180.0)

def gps_newpos(lat, lon, bearing, distance):
    '''extrapolate latitude/longitude given a heading and distance
    thanks to http://www.movable-type.co.uk/scripts/latlong.html
    '''
    import math
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    brng = math.radians(bearing)
    dr = distance/radius_of_earth
    
    lat2 = math.asin(math.sin(lat1)*math.cos(dr) +
                     math.cos(lat1)*math.sin(dr)*math.cos(brng))
    lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(dr)*math.cos(lat1),
                             math.cos(dr)-math.sin(lat1)*math.sin(lat2))
    return (math.degrees(lat2), wrap_valid_longitude(math.degrees(lon2)))

def vinc_dist(phi1,  lembda1,  phi2,  lembda2,   f = 1.0 / 298.257223563, a = 6378137.0) :
    """ 
    Returns the distance between two geographic points on the ellipsoid
    and the forward and reverse azimuths between these points.
    lats, longs and azimuths are in decimal degrees, distance in metres 
    
    Returns ( s, alpha12,  alpha21 ) as a tuple
    """
    
    if (abs( phi2 - phi1 ) < 1e-8) and ( abs( lembda2 - lembda1) < 1e-8 ) :
            return 0.0, 0.0, 0.0
    
    piD4   = math.atan( 1.0 )
    two_pi = piD4 * 8.0
    
    phi1    = phi1 * piD4 / 45.0
    lembda1 = lembda1 * piD4 / 45.0        # unfortunately lambda is a key word!
    phi2    = phi2 * piD4 / 45.0
    lembda2 = lembda2 * piD4 / 45.0
    
    b = a * (1.0 - f)
    
    TanU1 = (1-f) * math.tan( phi1 )
    TanU2 = (1-f) * math.tan( phi2 )
    
    U1 = math.atan(TanU1)
    U2 = math.atan(TanU2)
    
    lembda = lembda2 - lembda1
    last_lembda = -4000000.0        # an impossibe value
    omega = lembda
    
    # Iterate the following equations, 
    #  until there is no significant change in lembda 
    
    while ( last_lembda < -3000000.0 or lembda != 0 and abs( (last_lembda - lembda)/lembda) > 1.0e-9 ) :
    
        sqr_sin_sigma = pow( math.cos(U2) * math.sin(lembda), 2) + \
                pow( (math.cos(U1) * math.sin(U2) - \
                math.sin(U1) *  math.cos(U2) * math.cos(lembda) ), 2 )

        Sin_sigma = math.sqrt( sqr_sin_sigma )

        Cos_sigma = math.sin(U1) * math.sin(U2) + math.cos(U1) * math.cos(U2) * math.cos(lembda)

        sigma = math.atan2( Sin_sigma, Cos_sigma )

        Sin_alpha = math.cos(U1) * math.cos(U2) * math.sin(lembda) / math.sin(sigma)
        alpha = math.asin( Sin_alpha )

        Cos2sigma_m = math.cos(sigma) - (2 * math.sin(U1) * math.sin(U2) / pow(math.cos(alpha), 2) )

        C = (f/16) * pow(math.cos(alpha), 2) * (4 + f * (4 - 3 * pow(math.cos(alpha), 2)))

        last_lembda = lembda

        lembda = omega + (1-C) * f * math.sin(alpha) * (sigma + C * math.sin(sigma) * \
                (Cos2sigma_m + C * math.cos(sigma) * (-1 + 2 * pow(Cos2sigma_m, 2) )))
    
    u2 = pow(math.cos(alpha),2) * (a*a-b*b) / (b*b)
    
    A = 1 + (u2/16384) * (4096 + u2 * (-768 + u2 * (320 - 175 * u2)))
    
    B = (u2/1024) * (256 + u2 * (-128+ u2 * (74 - 47 * u2)))
    
    delta_sigma = B * Sin_sigma * (Cos2sigma_m + (B/4) * \
            (Cos_sigma * (-1 + 2 * pow(Cos2sigma_m, 2) ) - \
            (B/6) * Cos2sigma_m * (-3 + 4 * sqr_sin_sigma) * \
            (-3 + 4 * pow(Cos2sigma_m,2 ) )))
    
    s = b * A * (sigma - delta_sigma)
    
    alpha12 = math.atan2( (math.cos(U2) * math.sin(lembda)), \
            (math.cos(U1) * math.sin(U2) - math.sin(U1) * math.cos(U2) * math.cos(lembda)))
    
    alpha21 = math.atan2( (math.cos(U1) * math.sin(lembda)), \
            (-math.sin(U1) * math.cos(U2) + math.cos(U1) * math.sin(U2) * math.cos(lembda)))
    
    if ( alpha12 < 0.0 ) : 
            alpha12 =  alpha12 + two_pi
    if ( alpha12 > two_pi ) : 
            alpha12 = alpha12 - two_pi
    
    alpha21 = alpha21 + two_pi / 2.0
    if ( alpha21 < 0.0 ) : 
            alpha21 = alpha21 + two_pi
    if ( alpha21 > two_pi ) : 
            alpha21 = alpha21 - two_pi
    
    alpha12    = alpha12    * 45.0 / piD4
    alpha21    = alpha21    * 45.0 / piD4
    return s, alpha12,  alpha21 

# END of Vincenty's Inverse formulae 


#-------------------------------------------------------------------------------
# Vincenty's Direct formulae                            |
# Given: latitude and longitude of a point (phi1, lembda1) and             |
# the geodetic azimuth (alpha12)                         |
# and ellipsoidal distance in metres (s) to a second point,            |
#                                         |
# Calculate: the latitude and longitude of the second point (phi2, lembda2)     |
# and the reverse azimuth (alpha21).                        |
#                                         |
#-------------------------------------------------------------------------------

def  vinc_pt(phi1, lembda1, alpha12, s,  f = 1.0 / 298.257223563, a = 6378137.0) :
    """
    
    Returns the lat and long of projected point and reverse azimuth
    given a reference point and a distance and azimuth to project.
    lats, longs and azimuths are passed in decimal degrees
    
    Returns ( phi2,  lambda2,  alpha21 ) as a tuple 
    
    """
    
    piD4 = math.atan( 1.0 )
    two_pi = piD4 * 8.0
    
    phi1    = phi1    * piD4 / 45.0
    lembda1 = lembda1 * piD4 / 45.0
    alpha12 = alpha12 * piD4 / 45.0
    if ( alpha12 < 0.0 ) : 
            alpha12 = alpha12 + two_pi
    if ( alpha12 > two_pi ) : 
            alpha12 = alpha12 - two_pi
    
    b = a * (1.0 - f)
    
    TanU1 = (1-f) * math.tan(phi1)
    U1 = math.atan( TanU1 )
    sigma1 = math.atan2( TanU1, math.cos(alpha12) )
    Sinalpha = math.cos(U1) * math.sin(alpha12)
    cosalpha_sq = 1.0 - Sinalpha * Sinalpha
    
    u2 = cosalpha_sq * (a * a - b * b ) / (b * b)
    A = 1.0 + (u2 / 16384) * (4096 + u2 * (-768 + u2 * \
            (320 - 175 * u2) ) )
    B = (u2 / 1024) * (256 + u2 * (-128 + u2 * (74 - 47 * u2) ) )
    
    # Starting with the approximation
    sigma = (s / (b * A))
    
    last_sigma = 2.0 * sigma + 2.0    # something impossible
    
    # Iterate the following three equations 
    #  until there is no significant change in sigma 
    
    # two_sigma_m , delta_sigma
    while ( abs( (last_sigma - sigma) / sigma) > 1.0e-9 ) :
        two_sigma_m = 2 * sigma1 + sigma

        delta_sigma = B * math.sin(sigma) * ( math.cos(two_sigma_m) \
                + (B/4) * (math.cos(sigma) * \
                (-1 + 2 * math.pow( math.cos(two_sigma_m), 2 ) -  \
                (B/6) * math.cos(two_sigma_m) * \
                (-3 + 4 * math.pow(math.sin(sigma), 2 )) *  \
                (-3 + 4 * math.pow( math.cos (two_sigma_m), 2 ))))) \

        last_sigma = sigma
        sigma = (s / (b * A)) + delta_sigma
    
    phi2 = math.atan2 ( (math.sin(U1) * math.cos(sigma) + math.cos(U1) * math.sin(sigma) * math.cos(alpha12) ), \
            ((1-f) * math.sqrt( math.pow(Sinalpha, 2) +  \
            pow(math.sin(U1) * math.sin(sigma) - math.cos(U1) * math.cos(sigma) * math.cos(alpha12), 2))))
    
    lembda = math.atan2( (math.sin(sigma) * math.sin(alpha12 )), (math.cos(U1) * math.cos(sigma) -  \
            math.sin(U1) *  math.sin(sigma) * math.cos(alpha12)))
    
    C = (f/16) * cosalpha_sq * (4 + f * (4 - 3 * cosalpha_sq ))
    
    omega = lembda - (1-C) * f * Sinalpha *  \
            (sigma + C * math.sin(sigma) * (math.cos(two_sigma_m) + \
            C * math.cos(sigma) * (-1 + 2 * math.pow(math.cos(two_sigma_m),2) )))
    
    lembda2 = lembda1 + omega
    
    alpha21 = math.atan2 ( Sinalpha, (-math.sin(U1) * math.sin(sigma) +  \
            math.cos(U1) * math.cos(sigma) * math.cos(alpha12)))
    
    alpha21 = alpha21 + two_pi / 2.0
    if ( alpha21 < 0.0 ) :
            alpha21 = alpha21 + two_pi
    if ( alpha21 > two_pi ) :
            alpha21 = alpha21 - two_pi
    
    phi2       = phi2       * 45.0 / piD4
    lembda2    = lembda2    * 45.0 / piD4
    alpha21    = alpha21    * 45.0 / piD4
    
    return phi2,  lembda2,  alpha21 
    
    # END of Vincenty's Direct formulae