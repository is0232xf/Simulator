import time
import math
from Tools import ArduTools
from pymavlink import mavutil # Needed for command message definitions
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command



class ArduPilotDriver(object):

    AIR_SPEED = 3

    def __init__(self):
        self._IsEchoOn = False
        self._IsRunning = False
        self._IsConnected = False
        self._IsFlying = False

        # define connection method 
        # see all the methods at http://python.dronekit.io/guide/connecting_vehicle.html
        self.connection_string = "0.0.0.0:14551"
        self.vehicle = None    

    def sendEmergencyStop(self):

        print("Stop")

    def start(self):
        # Connect_IsEchoOn
        try:
            self.vehicle = connect(self.connection_string, wait_ready=True)
            self._IsConnected = True
            # set airspeed to 3 m/s
            self.vehicle.airspeed = self.AIR_SPEED
            print("Succeeded to connect")
        except:
            print("Failed to connect")
    def exit(self):
        if self._IsRunning:
            self.sendEmergencyStop()
            self._IsRunning = False
            self._IsFlying = False
            
            print("Close vehicle object")
            self.vehicle.close()

    def setAirSpeed(self, speed):
        self.AIR_SPEED = speed
        self.vehicle.airspeed = self.AIR_SPEED

    def setHomeLocation(self, Location = None):
        if Location == None:
            self.vehicle.home_location = self.vehicle.location.global_frame
        else:
            self.vehicle.home_location = Location


    ###############################################################################################
    #
    #   Control and Status functions
    #
    ###############################################################################################

    def isMoving(self):
        return any(math.fabs(i) >= 0.3 for i in self.vehicle.velocity)
    
    def isAirborne(self):
        print(self.vehicle.location.global_relative_frame.alt)
        return self.vehicle.location.global_relative_frame.alt > 0.05 and self.isReady()

    def isReady(self):
        return self.vehicle.system_status.state == 'ACTIVE' and self.vehicle.armed == True

    def isMode(self, mode):
        return self.vehicle.mode == mode

    def isActive(self, mode):
        return self._IsFlying and self.isReady() and self.isMode(mode)


    ###############################################################################################
    #
    #   Navigation functions
    #
    ###############################################################################################

    def takeoff(self, aTargetAltitude = 2):

        if  not self._IsConnected:
            print("Not connected")
            return
            
        if  self._IsFlying:
            print("Busy cant takeoff now")
            return

        if  self.isAirborne():
            print("Already on the air")
            return
        
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:      
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")

        self._IsFlying = True

        self.vehicle.simple_takeoff(aTargetAltitude)
        time.sleep(2)
        
        # Wait for takeoff to finish
        while self.isActive("GUIDED"):
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)

            if ArduTools.hasReachedAltitude(self.vehicle.location.global_relative_frame.alt, aTargetAltitude):
            #if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude*0.95: #Trigger just below target alt.
                print("Reached target altitude")
                break
            time.sleep(1)

        self._IsFlying = False

    def land(self):

        if not self.isAirborne():
            print("Not on the air")
            return

        print("Begin landing!")
        #Release any current task
        self._IsFlying = False
        time.wait(2)

        #Begin task
        self._IsFlying = True       
        
        self.vehicle.mode = VehicleMode("LAND")

        aTargetAltitude = 0

        while self.isActive("LAND"):

            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)    
            if ArduTools.hasReachedAltitude(self.vehicle.location.global_relative_frame.alt, aTargetAltitude):  
            #if self.vehicle.location.global_relative_frame.alt <= 0.05: #Trigger just below target alt.
                print("Landed")
                break
            time.sleep(1)

        #Wair for automatic disaming
        time.sleep(3)

        #Release current task
        self._IsFlying = False
        
    def returnToHome(self):

        self._IsFlying = False
        time.sleep(1)

        if(self.isAirborne() and self.isReady() and not self._IsFlying):
            self._IsFlying = True

            # set airspeed
            self.vehicle.airspeed = self.AIR_SPEED
            # set TRL mode to return
            self.vehicle.mode = VehicleMode("RTL")
            time.sleep(2)
            # create gps location point
            target_location = self.vehicle.home_location
            print(target_location)
            
            while self.isActive("RTL"):
                if target_location == None or self.vehicle.location.global_frame == None:
                    print("global_frame", self.vehicle.location.global_frame)
                    time.sleep(1)
                    continue
                
                # target_distance = ArduTools.get_distance_metres(self.vehicle.location.global_frame, target_location)
                # print("target_distance", target_distance)

                remainingDistance = ArduTools.get_distance_metres(self.vehicle.location.global_frame, target_location)
                print("Distance to target: ", remainingDistance)

                if ArduTools.hasReachedDestination(self.vehicle.location.global_frame, target_location):
                    print("Reached target")
                    break

                time.sleep(1)
                
            while self.isActive("RTL"):
                print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)      

                if self.vehicle.location.global_relative_frame.alt <= 0.05: #Trigger just below target alt.
                    print("Landed")
                    break

                time.sleep(1)
        
            self._IsFlying = False
        else:
            print("Not on the air")
       
    def nav_change_heading(self, angle, overide = False):

        if(not self.isAirborne() or not self.isReady() or (self._IsFlying and not overide)):
            print("Not airbourne")
        
        if not overide:
            self._IsFlying = True

        direction = 1 if angle > 0 else -1

        heading = math.fabs(angle)

        target = self.vehicle.heading + angle

        target = target if target < 360 else target - 360
        target = target if target > 0 else target + 360

        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,       # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0,          #confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            direction,  # param 3, direction -1 ccw, 1 cw
            1,          # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

        # Wait for takeoff to finish
        print("Target heading ", target)

        while self.isActive("GUIDED"):
            print(" Heading: ", self.vehicle.heading)    
            if ArduTools.hasReachedHeading(self.vehicle.heading, target):
                print("Reached heading ", self.vehicle.heading)
                break
            time.sleep(1)
        
        if not overide:
            self._IsFlying = False

    def nav_goto_heading(self, heading, cw = True, overide = False):

        if(not self.isAirborne() or not self.isReady() or (self._IsFlying and not overide)):
            print("Not airbourne")
        
        if not overide:
            self._IsFlying = True

        direction = 1 if cw else -1

        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,       # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0,          #confirmation
            heading,      # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            direction,  # param 3, direction -1 ccw, 1 cw
            0,          # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

        print("Target heading ", heading)
        while self.isActive("GUIDED"):
            print(" Heading: ", self.vehicle.heading)
            if ArduTools.hasReachedHeading(self.vehicle.heading, heading):
                print("Reached heading ", self.vehicle.heading)
                break
            time.sleep(1)
        
        if not overide:
            self._IsFlying = False

    def nav_change_altitute(self, altitude):

        if(not self.isAirborne() or not self.isReady() or self._IsFlying):
            print("Not armed or in GUIDED mode")
        
        self._IsFlying = True

        current_loc = self.vehicle.location.global_relative_frame #get current location
        aTargetAltitude = current_loc.alt + altitude
        target_loc = LocationGlobalRelative(current_loc.lat, current_loc.lon, aTargetAltitude)
            
        self.vehicle.simple_goto(target_loc)

        while self.isActive("GUIDED"):
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)

            if ArduTools.hasReachedAltitude(self.vehicle.location.global_relative_frame.alt, aTargetAltitude):
                print("Reached target altitude ", self.vehicle.location.global_relative_frame.alt)
                break
            time.sleep(1)
        
        self._IsFlying = False
             

    def nav_goto_altitute(self, aTargetAltitude):

        if(not self.isAirborne() or not self.isReady() or self._IsFlying):
            print("Not airbourne")
        
        self._IsFlying = True

        current_loc = self.vehicle.location.global_relative_frame #get current location
        target_loc = LocationGlobalRelative(current_loc.lat, current_loc.lon, aTargetAltitude)
            
        self.vehicle.simple_goto(target_loc)

        while self.isActive("GUIDED"):
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)     
            if ArduTools.hasReachedAltitude(self.vehicle.location.global_relative_frame.alt, aTargetAltitude):
                print("Reached target altitude ", self.vehicle.location.global_relative_frame.alt)
                break
            time.sleep(1)
        
        self._IsFlying = False

    def nav_goto_gps_point(self, target_location, alt = 0):

        if(not self.isAirborne() or not self.isReady() or self._IsFlying):
            print("Not airbourne")
        
        self._IsFlying = True

        # set airspeed
        self.vehicle.airspeed = self.AIR_SPEED
        # if altitude is not especified then get current altitude
        current_altitude = self.vehicle.location.global_relative_frame.alt if alt == 0 else alt
        # create gps location point
        #target_location = LocationGlobalRelative(lat, long, current_altitude)
        #target_distance = ArduTools.get_distance_metres(self.vehicle.location.global_frame, target_location)

        self.nav_goto_heading(ArduTools.get_bearing(self.vehicle.location.global_frame, target_location), overide = True)

        self.vehicle.simple_goto(target_location)

        time.sleep(2)

        while self.isActive("GUIDED") and self.isMoving():
            remainingDistance = ArduTools.get_distance_metres(self.vehicle.location.global_frame, target_location)
            print("Distance to target: ", remainingDistance)

            if ArduTools.hasReachedDestination(self.vehicle.location.global_frame, target_location):
                print("Reached target")
                break
            time.sleep(1)

        
        self._IsFlying = False

    def nav_goto_gps_coordinates(self, lat, long, alt = 0):

        if(not self.isAirborne() or not self.isReady() or self._IsFlying):
            print("Not airbourne")
        
        self._IsFlying = True

        # set airspeed
        self.vehicle.airspeed = self.AIR_SPEED
        # if altitude is not especified then get current altitude
        current_altitude = self.vehicle.location.global_relative_frame.alt if alt == 0 else alt
        # create gps location point
        target_location = LocationGlobalRelative(lat, long, current_altitude)
        target_distance = ArduTools.get_distance_metres(self.vehicle.location.global_frame, target_location)
        self.vehicle.simple_goto(target_location)

        while self.isActive("GUIDED"):
            remainingDistance = ArduTools.get_distance_metres(self.vehicle.location.global_frame, target_location)
            print("Distance to target: ", remainingDistance)

            if ArduTools.hasReachedDestination(self.vehicle.location.global_frame, target_location):
                print("Reached target")
                break
            time.sleep(1)

        
        self._IsFlying = False

    def nav_goto_gps_xy_NE_coordinates(self, x, y, alt = 0):

        if(not self.isAirborne() or not self.isReady() or self._IsFlying):
            print("Not airbourne")
        
        self._IsFlying = True

        # set airspeed
        self.vehicle.airspeed = self.AIR_SPEED
        # if altitude is not especified then get current altitude
        current_altitude = self.vehicle.location.global_relative_frame.alt if alt == 0 else alt
        # create gps location point
        target_location = ArduTools.get_location_metres(self.vehicle.location.global_frame, x, y, alt)
        target_distance = ArduTools.get_distance_metres(self.vehicle.location.global_frame, target_location)

        self.nav_goto_heading( ArduTools.get_bearing(self.vehicle.location.global_frame, target_location), overide = True)

        self.vehicle.simple_goto(target_location)

        while self.isActive("GUIDED"):
            remainingDistance = ArduTools.get_distance_metres(self.vehicle.location.global_frame, target_location)
            print("Distance to target: ", remainingDistance)

            if ArduTools.hasReachedDestination(self.vehicle.location.global_frame, target_location):
                print("Reached target")
                break
            time.sleep(1)
        
        self._IsFlying = False

    def nav_goto_gps_xy_body_coordinates(self, x, y, alt = 0):

        if(not self.isAirborne() or not self.isReady() or self._IsFlying):
            print("Not airbourne")
        
        self._IsFlying = True

        # set airspeed
        self.vehicle.airspeed = self.AIR_SPEED
        # if altitude is not especified then get current altitude
        current_altitude = self.vehicle.location.global_relative_frame.alt if alt == 0 else alt
        # create gps location point
        target_location = ArduTools.get_location_metres_local(self.vehicle.location.global_frame, self.vehicle.heading, x, y, alt)
        target_distance = ArduTools.get_distance_metres(self.vehicle.location.global_frame, target_location)

        self.nav_goto_heading(ArduTools.get_bearing(self.vehicle.location.global_frame, target_location), overide = True)

        self.vehicle.simple_goto(target_location)

        while self.isActive("GUIDED"):
            remainingDistance = ArduTools.get_distance_metres(self.vehicle.location.global_frame, target_location)
            print("Distance to target: ", remainingDistance)

            if ArduTools.hasReachedDestination(self.vehicle.location.global_frame, target_location):
                print("Reached target")
                break
            time.sleep(1)
        
        self._IsFlying = False
        
    def nav_move_velocity_pos(self, velocity_x, velocity_y, velocity_z, duration = 0):
        """
        Move vehicle in direction based on specified velocity vectors.
        The EKF origin is the vehicles location when it first achieved a good position estimate
        North (x), East(y), Down(z)
        """   

        if(not self.isAirborne() or not self.isReady() or self._IsFlying):
            print("Not airbourne")
        
        self._IsFlying = True

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        if duration > 0:
            # send command to vehicle on 1 Hz cycle
            for x in range(0, duration):
                self.vehicle.send_mavlink(msg)
                time.sleep(0.5)
        else:
            self.vehicle.send_mavlink(msg)
            time.sleep(0.5)
        
        self.vehicle.flush()
        
        self._IsFlying = False

    def nav_move_velocity_body(self, velocity_x, velocity_y, velocity_z, duration = 0):
        """
        Move vehicle in direction based on specified velocity vectors.
        Velocities are relative to the current vehicle heading. Use this to specify the speed forward, right and down (or the opposite if you use negative values).
        Forward (x), Right(y), Down(z)
        """
        
        if(not self.isAirborne() or not self.isReady() or self._IsFlying):
            print("Not airbourne")
        
        self._IsFlying = True

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        if duration > 0:
            # send command to vehicle on 1 Hz cycle
            for x in range(0, duration):
                self.vehicle.send_mavlink(msg)
                time.sleep(0.5)
        else:
            self.vehicle.send_mavlink(msg)
            time.sleep(0.5)
        
        self.vehicle.flush()
        
        self._IsFlying = False

    def nav_move_distance_pos(self, distance_x, distance_y, distance_z):
        """
        Move vehicle in direction based on specified velocity vectors.
        The EKF origin is the vehicles location when it first achieved a good position estimate
        North (x), East(y), Down(z)
        """  

        if(not self.isAirborne() or not self.isReady() or self._IsFlying):
            print("Not airbourne")
        
        self._IsFlying = True
             
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, # frame
            0b0000111111111000, # type_mask (only speeds enabled)
            distance_x, distance_y, distance_z, # x, y, z positions (not used)
            0, 0, 0, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

        time.sleep(2)

        while self.isMoving():
            time.sleep(1)
        
        self._IsFlying = False

    def nav_move_distance_body(self, distance_x, distance_y, distance_z):
        """
        Move vehicle in direction based on specified velocity vectors.
        Velocities are relative to the current vehicle heading. Use this to specify the speed forward, right and down (or the opposite if you use negative values).
        Positions are relative to the vehicles current position and heading
        Forward (x), Right(y), Down(z)
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
            0b0000111111111000, # type_mask (only speeds enabled)
            distance_x, distance_y, distance_z, # x, y, z positions (not used)
            0, 0, 0, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

        time.sleep(2)

        while self.isMoving():
            time.sleep(1)
        
        self._IsFlying = False

    ###############################################################################################
    #
    #   NO GPS Navigation functions
    #
    ###############################################################################################

    # Turn right: set_attitude(roll_angle = 1, thrust = 0.5, duration = 3)
    # Sping right: set_attitude(yaw_rate = 30, thrust = 0.5, duration = 3)
    # Move forward: set_attitude(pitch_angle = -5, thrust = 0.5, duration = 3)
    # Move backward: set_attitude(pitch_angle = 5, thrust = 0.5, duration = 3)   
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend

    def arm_and_takeoff_nogps(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude without GPS data.
        """

        ##### CONSTANTS #####
        DEFAULT_TAKEOFF_THRUST = 0.7
        SMOOTH_TAKEOFF_THRUST = 0.6

        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        # If you need to disable the arming check,
        # just comment it with your own responsibility.
        while not vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED_NOGPS mode
        self.vehicle.mode = VehicleMode("GUIDED_NOGPS")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print(" Waiting for arming...")
            self.vehicle.armed = True
            time.sleep(1)

        print("Taking off!")

        thrust = DEFAULT_TAKEOFF_THRUST
        while True:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            print(" Altitude: %f  Desired: %f" %
                (current_altitude, aTargetAltitude))
            if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
                print("Reached target altitude")
                break
            elif current_altitude >= aTargetAltitude*0.6:
                thrust = SMOOTH_TAKEOFF_THRUST
            self.setAttitude(thrust = thrust)
            time.sleep(0.2)


    def setAttitude(self, roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0):
        """
        Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
        with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
        velocity persists until it is canceled. The code below should work on either version
        (sending the message multiple times does not cause problems).
        """
        
        """
        The roll and pitch rate cannot be controllbed with rate in radian in AC3.4.4 or earlier,
        so you must use quaternion to control the pitch and roll for those vehicles.
        """
        
        # Thrust >  0.5: Ascend
        # Thrust == 0.5: Hold the altitude
        # Thrust <  0.5: Descend
        msg = self.vehicle.message_factory.set_attitude_target_encode(
            0, # time_boot_ms
            1, # Target system
            1, # Target component
            0b00000000, # Type mask: bit 1 is LSB
            ArduTools.to_quaternion(roll_angle, pitch_angle), # Quaternion
            0, # Body roll rate in radian
            0, # Body pitch rate in radian
            math.radians(yaw_rate), # Body yaw rate in radian
            thrust  # Thrust
        )
        self.vehicle.send_mavlink(msg)

        start = time.time()
        while time.time() - start < duration:
            self.vehicle.send_mavlink(msg)
            time.sleep(0.1)


    ###############################################################################################
    #
    #   Auto-Navigation functions
    #
    ###############################################################################################
    def start_autonavigation(self):
        print("Starting mission")
        # Reset mission set to first (0) waypoint
        self.vehicle.commands.next=0

        # Set mode to AUTO to start mission
        self.vehicle.mode = VehicleMode("AUTO")


        # Monitor mission. 
        # Demonstrates getting and setting the command number 
        # Uses distance_to_current_waypoint(), a convenience function for finding the 
        #   distance to the next waypoint.

        while True:
            nextwaypoint=self.vehicle.commands.next
            print("Distance to waypoint (%s): %s" % (nextwaypoint, distance_to_current_waypoint()))
        
            if nextwaypoint==3: #Skip to next waypoint
                print("Skipping to Waypoint 5 when reach waypoint 3")
                self.vehicle.commands.next = 5
            if nextwaypoint==5: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
                print("Exit 'standard' mission when start heading to final waypoint (5)")
                break
            time.sleep(1)

    def download_mission(self):
        """
        Download the current mission from the vehicle.
        """
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready() # wait until download is complete.
        
    def distance_to_current_waypoint(self):
        """
        Gets distance in metres to the current waypoint. 
        It returns None for the first waypoint (Home location).
        """
        nextwaypoint = self.vehicle.commands.next
        if nextwaypoint==0:
            return None
        missionitem=self.vehicle.commands[nextwaypoint-1] #commands are zero indexed
        lat = missionitem.x
        lon = missionitem.y
        alt = missionitem.z
        targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
        distancetopoint = get_distance_metres(self.vehicle.location.global_frame, targetWaypointLocation)
        return distancetopoint

    def adds_square_mission(self, aLocation, aSize):
        """
        Adds a takeoff command and four waypoint commands to the current mission. 
        The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

        The function assumes vehicle.commands matches the vehicle mission state 
        (you must have called download at least once in the session and after clearing the mission)
        """	

        cmds = self.vehicle.commands

        print(" Clear any existing commands")
        cmds.clear() 
        
        print(" Define/add new commands.")
        # Add new commands. The meaning/order of the parameters is documented in the Command class. 
        
        #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

        #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
        point1 = get_location_metres(aLocation, aSize, -aSize)
        point2 = get_location_metres(aLocation, aSize, aSize)
        point3 = get_location_metres(aLocation, -aSize, aSize)
        point4 = get_location_metres(aLocation, -aSize, -aSize)
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 11))
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 12))
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 13))
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))
        #add dummy waypoint "5" at point 4 (lets us know when have reached destination)
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))    

        print(" Upload new commands to vehicle")
        cmds.upload()