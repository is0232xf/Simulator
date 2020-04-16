**HOW TO USE (Simulation)**

*File description*

- create_waypoint_data.py: create waypoint data. 
	- input: NONE, output: waypoint data(.csv) 
- controller/main.py: simulate the device behaivior along with waypoint data. 
	- input: waypoint data(.csv), output: simulated data(.csv), perfomace data(.png) (if file_log == True)

*Operation*
1. Search set of longitudes and latitudes of waypoints on Google map. 
1. Create the waypoint file(.csv).
	1. `python create_waypoint_data.py`
1. Simulate the device behavior and create the tracking data(.csv).
	1. `python controller/main.py`
1. If file log mode is on(able to configure in main.py), save the device behavior as a csv data & sequential figure(.png) automatically.

*Example*

Square waypoint

![square](https://user-images.githubusercontent.com/17609665/69784257-b7b6e600-11f8-11ea-9299-f10697b01114.png)

Star waypoint

![star](https://user-images.githubusercontent.com/17609665/69784283-c69d9880-11f8-11ea-8cec-ea84df23ae49.png)

_Future work_

##### * Enable to input data from the teminal.
