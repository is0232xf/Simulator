#import rospy
import time
import math

from dronekit import LocationGlobal, LocationGlobalRelative

class ArduTools:

    @staticmethod
    def hasReachedHeading(current_heading, current_target, margin = 1):
        
        current = 0 if current_heading == 360 else current_heading
        target = 0 if current_target == 360 else current_target

        lower_limit = ArduTools.safeAddTo360(target, -margin)
        upper_limit = ArduTools.safeAddTo360(target, margin)

        if (upper_limit - lower_limit) == (2 * margin):
            return current >= lower_limit and current <= upper_limit
        else:
            return (current >= lower_limit and current <= 359) or (current >= 0 and current <= upper_limit)

    @staticmethod
    def hasReachedAltitude(current_altitude, target_altitude, margin = 0.005):

        current = current_altitude

        lower_limit = target_altitude * (1 - margin)
        upper_limit = target_altitude * (1 + margin)

        return current >= lower_limit and current <= upper_limit

    @staticmethod
    def hasReachedDestination(current_location, target_location, margin = 1):

        remainingDistance = ArduTools.get_distance_metres(current_location, target_location)
        
        return remainingDistance <= margin

    @staticmethod
    def safeAddTo360(value, increment):

        value = value + increment

        value = value - 360 if value >= 360 else value

        value = value + 360 if value < 0 else value

        return value

    @staticmethod
    def get_distance_metres(aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.

        This method is an approximation, and will not be accurate over large distances and close to the 
        earth's poles. It comes from the ArduPilot test code: 
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

    @staticmethod
    def get_location_metres(original_location, dNorth, dEast, alt = 0):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
        specified `original_location`. The returned Location has the same `alt` value
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to 
        the current vehicle position.
        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius=6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)

        altitude = original_location.alt if alt == 0 else alt
        if altitude is None:
            altitude = 0
            print("altitude: ", altitude)
        location = LocationGlobal(newlat, newlon, altitude)
        return LocationGlobal(newlat, newlon, altitude)

    @staticmethod
    def get_location_metres_local(original_location, heading, x, y, alt = 0):
        """
        Returns a LocationGlobal object containing the latitude/longitude `x` and `y` metres relative to the body from the 
        specified `original_location`. The returned Location has the same `alt` value
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to 
        the current vehicle position and bearing.
        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """

        #Convert body x, y coordinates to NE coordinates to add them to current position
        dNorth, dEast = ArduTools.rotate_body_frame_to_NE(heading, x, y)

        earth_radius=6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)

        altitude = original_location.alt if alt == 0 else alt

        return LocationGlobal(newlat, newlon, altitude)
    
    @staticmethod
    def get_bearing(aLocation1, aLocation2):
        """
        Returns the bearing between the two LocationGlobal objects passed as parameters.
        This method is an approximation, and may not be accurate over large distances and close to the 
        earths poles. It comes from the ArduPilot test code: 
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """	
        off_x = aLocation2.lon - aLocation1.lon
        off_y = aLocation2.lat - aLocation1.lat
        bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
        if bearing < 0:
            bearing += 360.00

        return round(bearing)

    @staticmethod
    def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
        """
        Convert degrees to quaternions
        """
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5

        return [w, x, y, z]

    @staticmethod
    def rotate_body_frame_to_NE(heading, x, y):
        """
        Convert x,y relative coordinates to x,y NE coordinates
        x points to the head of the drone or in the same direccion as the heading
        y points to the right side of the drone or +90 degrees from the heading
        """
        c = math.cos(math.radians(heading))
        s = math.sin(math.radians(heading))

        ne_x = round(x * c - y * s, 6)
        ne_y = round(x * s + y * c, 6)

        return ne_x, ne_y

    @staticmethod
    def calculate_boustrophedon_points(len = 40, coverage = 2):
        P = []
        P0 = (0,0)

        #Total points
        #T = int((2 * radius) / coverage)
        T = int(len / coverage)

        #Lenght of the longest straight line path
        #x_d = 2 * radius - coverage
        x_d = len - coverage
        y_d = coverage
        
        for i in range(2 * T):
            if i == 0:
                Pt = (P0[0] - x_d/2, P0[1] - x_d/2)
                P.append(Pt) #add initial point
                M = 1 #Magnitude (1 or -1)
                continue
                
            if i % 2 == 1: #X displacement point
                Pt = (P[i - 1][0] + (M * x_d), P[i-1][1])
                M = -M #Change direction on X axis
            else: #Y displacement point
                Pt = (P[i-1][0], P[i-1][1] + y_d)
            #Add point to array
            P.append(Pt)
        
        return P
