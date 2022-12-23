#!/usr/bin/env python
# license removed for brevity
import importlib
import rospy
import socket
from io import BytesIO
from mavros_msgs.srv import VehicleInfoGet, VehicleInfoGetResponse, VehicleInfoGetRequest
from mavros_msgs.msg import HomePosition
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from geographic_msgs.msg import GeoPose, GeoPoint, GeoPath
from sensor_msgs.msg import NavSatFix
from iq_gnc.msg import DroneStatusGlobal, DroneListGlobal, DroneStatusLocal, DroneListLocal
import geonav_transform.geonav_conversions as gc
import alvinxy.alvinxy as axy
importlib.reload(gc)
importlib.reload(axy)
bufferSize          = 1024
home_position :HomePosition
drone_dict_local = {}
"""key - sysid of drone, data - DroneStatusLocal
"""
drone_dict_global = {}
"""key - sysid of drone, data - DroneStatusGlobal
"""
def ds_global2local(ds_global:DroneStatusGlobal):
    ds_local= DroneStatusLocal(sysid=ds_global.sysid)
    xa, ya = gc.ll2xy(ds_global.position.latitude, ds_global.position.longitude,
        home_position.geo.latitude, home_position.geo.longitude)
    za = ds_global.position.altitude - home_position.geo.altitude
    ds_local.position.x = xa
    ds_local.position.y = ya
    ds_local.position.z = za
    return ds_local
def update_drone_list(ds_global:DroneStatusGlobal):
    ds_local = ds_global2local(ds_global)
    drone_dict_global[ds_global.sysid]=ds_global
    drone_dict_local[ds_local.sysid]=ds_local
def serialized_send(drone_status):
    serialize_buff = BytesIO()
    drone_status.serialize(serialize_buff)
    serialized_bytes = serialize_buff.getvalue()
    UDPClientSocket.sendto(serialized_bytes, serverAddressPort)
def serialized_recv():
    msgFromServer = UDPClientSocket.recvfrom(bufferSize)
    serialized_bytes = msgFromServer[0]
    ds=DroneStatusGlobal()
    ds.deserialize(serialized_bytes)
    #rospy.loginfo(rospy.get_caller_id() + "recv %s", ds)
    return ds
def publish_communication():
    drone_list_global_publish = DroneListGlobal()
    drone_list_local_publish = DroneListLocal()
    drone_list_global_publish.drones=drone_dict_global.values()
    drone_list_local_publish.drones=drone_dict_local.values()
    pub_global.publish(drone_list_global_publish)
    pub_local.publish(drone_list_local_publish)
def publish_self(ds_global : DroneStatusGlobal()):
    ds_local = ds_global2local(ds_global)
    pub_self_global.publish(ds_global)
    pub_self_local.publish(ds_local)
def callback_home_position(data :HomePosition):
    global home_position 
    home_position = data

def callback(data :NavSatFix):
    rospy.loginfo(rospy.get_caller_id() + " sysid %s len_list  %s",  sysid, len(drone_dict_global))
    my_gp = GeoPoint()
    my_gp.latitude  = data.latitude
    my_gp.longitude = data.longitude
    my_gp.altitude  = data.altitude
    
    ds = DroneStatusGlobal()
    ds.position=my_gp
    ds.sysid=sysid
    serialized_send(ds)
    #rospy.loginfo(rospy.get_caller_id() + "sent %s ",  ds)
    
    try:
        while True:
            ds_r = serialized_recv()
            update_drone_list(ds_r)
    except socket.error:
        pass

    publish_communication()
    publish_self(ds)

def talker():
    global ns
    global pub_global
    global pub_local
    global pub_self_global
    global pub_self_local
    global info_client
    global sysid
    global UDPClientSocket
    global serverAddressPort
    rospy.init_node('communication', anonymous=True)
    node_name = rospy.get_name()
    ip=rospy.get_param(node_name+"/ip")
    port=int(rospy.get_param(node_name+"/port"))
    serverAddressPort = (ip,port)
    rospy.loginfo(rospy.get_caller_id() + " server set to %s", serverAddressPort)
    UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    UDPClientSocket.setblocking(0)
    ns=rospy.get_namespace()
    pub_global = rospy.Publisher('{}communication/global'.format(ns), DroneListGlobal, queue_size=1)
    pub_local = rospy.Publisher('{}communication/local'.format(ns), DroneListLocal, queue_size=1)
    pub_self_global = rospy.Publisher('{}communication/self/global'.format(ns), DroneStatusGlobal, queue_size=1)
    pub_self_local = rospy.Publisher('{}communication/self/local'.format(ns), DroneStatusLocal, queue_size=1)
    
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
    rospy.Subscriber("{}mavros/home_position/home".format(ns), HomePosition, callback_home_position)
    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass