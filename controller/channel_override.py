from __future__ import print_function
from dronekit import connect

connection_string = "0.0.0.0:14551"
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

# Get all original channel values (before override)
print("Channel values from RC Tx:", vehicle.channels)

# Access channels individually
print("Read channels individually:")
print(" Ch1: %s" % vehicle.channels['1'])
print(" Ch2: %s" % vehicle.channels['2'])
print(" Ch3: %s" % vehicle.channels['3'])
print(" Ch4: %s" % vehicle.channels['4'])
print(" Ch5: %s" % vehicle.channels['5'])
print(" Ch6: %s" % vehicle.channels['6'])
print(" Ch7: %s" % vehicle.channels['7'])
print(" Ch8: %s" % vehicle.channels['8'])
print("Number of channels: %s" % len(vehicle.channels))


# Override channels
print("\nChannel overrides: %s" % vehicle.channels.overrides)

print("Set Ch4 override to 1700 (indexing syntax)")
vehicle.channels.overrides['4'] = 1700
print(" Channel overrides: %s" % vehicle.channels.overrides)
print(" Ch2 override: %s" % vehicle.channels.overrides['4'])

print("Set Ch5 override to 1300 (indexing syntax)")
vehicle.channels.overrides['5'] = 1300
print(" Channel overrides: %s" % vehicle.channels.overrides)
print(" Ch2 override: %s" % vehicle.channels.overrides['5'])

print("\nClose vehicle object")
vehicle.close()

print("Completed")