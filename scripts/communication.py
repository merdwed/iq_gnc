#!/usr/bin/env python
# license removed for brevity
import rospy
import socket
from io import BytesIO
from mavros_msgs.srv import VehicleInfoGet, VehicleInfoGetResponse, VehicleInfoGetRequest
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from geographic_msgs.msg import GeoPose, GeoPoint, GeoPath
from sensor_msgs.msg import NavSatFix
from iq_gnc.msg import DroneStatus, DroneList
serverAddressPort   = ("127.0.0.1", 20001)
bufferSize          = 1024
drone_list = []

def update_drone_list(drone_status:DroneStatus):
    for drone_s in drone_list:
        if drone_status.sysid == drone_s.sysid:
            drone_s.position = drone_status.position
            return
    drone_list.append(drone_status)
def serialized_send(drone_status):
    serialize_buff = BytesIO()
    drone_status.serialize(serialize_buff)
    serialized_bytes = serialize_buff.getvalue()
    UDPClientSocket.sendto(serialized_bytes, serverAddressPort)
def serialized_recv():
    msgFromServer = UDPClientSocket.recvfrom(bufferSize)
    serialized_bytes = msgFromServer[0]
    ds=DroneStatus()
    ds.deserialize(serialized_bytes)
    rospy.loginfo(rospy.get_caller_id() + "recv %s", ds)
    return ds
def callback(data :NavSatFix):
    rospy.loginfo(rospy.get_caller_id() + " sysid %s len_list  %s",  sysid, len(drone_list))
    gp = GeoPoint()
    gp.latitude  = data.latitude
    gp.longitude = data.longitude
    gp.altitude  = data.altitude
    
    ds = DroneStatus()
    ds.position=gp
    ds.sysid=sysid
    serialized_send(ds)
    rospy.loginfo(rospy.get_caller_id() + "sent %s ",  ds)
    
    try:
        while True:
            ds_r = serialized_recv()
            update_drone_list(ds_r)
    except socket.error:
        pass
    drone_list_publish = DroneList()
    drone_list_publish.drones=drone_list
    pub.publish(drone_list_publish)

def talker():
    global ns
    global pub
    global info_client
    global sysid
    global UDPClientSocket
    UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    UDPClientSocket.setblocking(0)
    ns=rospy.get_namespace()
    pub = rospy.Publisher('{}communication/data'.format(ns), DroneList, queue_size=1)
    rospy.init_node('communicator', anonymous=True)
    rospy.wait_for_service("{}mavros/vehicle_info_get".format(ns))
    info_client = rospy.ServiceProxy(
        name="{}mavros/vehicle_info_get".format(ns), 
        service_class=VehicleInfoGet
        )
    request = VehicleInfoGetRequest(0,0,0)
    response: VehicleInfoGetResponse
    response = info_client(request)
    sysid = response.vehicles[0].sysid
    rospy.Subscriber("{}mavros/global_position/global".format(ns), NavSatFix, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass