import rospy
import os
import asyncio
import json

# azure libraries
from azure.iot.device.aio import IoTHubDeviceClient

# ROS msg libraries
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

gps_long, gps_lat, gps_rel_alt, gps_hdg = 0
vel_x, vel_y, vel_z = 0

def gps_callback(gps_data):
  global gps_long, gps_lat
  gps_long = gps_data.longitude
  gps_lat = gps_data.latitude

def alt_callback(gps_rel_alt_data):
  global gps_rel_alt
  gps_rel_alt = gps_rel_alt_data.data
    
def gps_hdg_callback(gps_hdg_data):
  global gps_hdg
  gps_hdg = gps_hdg_data.data

def vel_callback(vel_data):
  global vel_x, vel_y, vel_z
  vel_x = vel_data.twist.linear.x
  vel_y = vel_data.twist.linear.y
  vel_z = vel_data.twist.linear.z

async def main():

    rospy.init_node('azure_send_hub')

    # Fetch the connection string from an enviornment variable
    conn_str = "HostName=rasendriyaHub.azure-devices.net;DeviceId=rizkyDesktop;SharedAccessKey=BrisCESDWi7U7h+RmTpaeimzUwtEE/YViQPxfkt5Cko="
    #os.getenv("IOTHUB_DEVICE_CONNECTION_STRING")

    # Create instance of the device client using the connection string
    device_client = IoTHubDeviceClient.create_from_connection_string(conn_str)

    rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_callback)
    rospy.Subscriber("/mavros/global_position/rel_alt", Float64, alt_callback)
    rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, gps_hdg_callback)
    rospy.Subscriber("/mavros/global_position/gp_vel", TwistStamped, vel_callback)

    # Connect the device client.
    await device_client.connect()

    rate = rospy.Rate(1)
    
    data_cloud = {}

    while not rospy.is_shutdown():
        data_cloud['Altitude'] = gps_rel_alt
        data_cloud['Speed'] = {'Speed' : vel_x, 'Lateral Speed' : vel_y, 'Vertical Speed' : vel_z}
        data_cloud['GPS'] = {'Heading': gps_hdg, 'Latitude' : gps_lat, 'Longitude' : gps_long}
        data_cloud_body = json.dumps(data_cloud)
        #print("Sending message: ", data_cloud_body)
        await device_client.send_message(data_cloud_body)
        rate.sleep()
        
    # Finally, shut down the client
    await device_client.shutdown()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except rospy.ROSInterruptException:
            pass