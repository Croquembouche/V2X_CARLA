from lib2to3.pytree import convert
from zmq import NULL
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from ast import walk
from distutils.spawn import spawn
import glob
import os
import sys
import carla
import numpy as np
import random
import math
import datetime
import cv2 as cv
import queue
import pymap3d as pm
import json

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ----------MSGs--------
from v2x_msg.msg import * 
from v2x_msg.msg import MAP, IntersectionGeometry, IntersectionReferenceID, Position3D, GenericLane, NodeListXY, NodeXY, NodeOffsetPointXY, Nodellmd64b, LaneAttributes
from v2x_msg.msg import (
    SPAT, IntersectionState, IntersectionReferenceID,
    IntersectionStatusObject, MovementState, MovementEvent, TimeChangeDetails
)

CARLA_TO_J2735 = {
    "Red": 3,      # stop-Then-Proceed
    "Yellow": 4,   # caution-Conflicting-Traffic
    "Green": 5     # protected-Movement-Allowed
}
from sensor_msgs.msg import Image

class DefaultInfrastructure(Node):
    def __init__(self, world):
        super().__init__('SPAT_Publisher')
        self.world = world
        self.blueprint_library = world.get_blueprint_library()
        self.bridge = CvBridge()
        self.sensors = []
        timer_period = 0.1  # seconds
        self.image_queue = queue.Queue()
        self.SPAT_publishers_ = []
        self.MAP_publishers_ = []
        self.prepMAPfromJson("/media/william/blueicedrive/Github/V2X_CARLA/src/MAPData/Inspiration_1743.json")
        self.timer = self.create_timer(timer_period, self.SPATCallback)
        self.timer = self.create_timer(timer_period, self.MAPCallback)
        self.camera_publisher_timer_ = self.create_timer(timer_period, self.RGBSensorcallback)
        self.giveCameras()
        # print(self.sensors)

# ---------------------------Helper Functions-------------------------

    def getTimeStamp(self):
        now = datetime.datetime.now()
        month_days = [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]

        # Handle leap year
        if (now.year % 4 == 0 and now.year % 100 != 0) or (now.year % 400 == 0):
            month_days[1] = 29

        # Calculate total days before this month
        days_before = sum(month_days[:now.month - 1])

        # Total minutes of the year
        total_minutes = (
            now.minute +
            now.hour * 60 +
            (now.day - 1 + days_before) * 24 * 60
        )

        # Milliseconds (including microsecond precision)
        milliseconds = now.second * 1000 + now.microsecond // 1000

        return total_minutes, milliseconds
    
    def convertCARLAIMGtoROSIMG(self, original):

        array = np.frombuffer(original.raw_data, dtype=np.dtype("uint8")) 
        array = np.reshape(array, (original.height, original.width, 4)) # RGBA
        image_message = self.bridge.cv2_to_imgmsg(array, encoding="passthrough")
        return image_message


# ---------------------------MAP--------------------------------

    def prepMAPfromJson(self, json_path):
        with open(json_path, 'r') as f:
            data = json.load(f)

        map_msg = MAP()
        map_msg.timestamp = self.getTimeStamp()[0]
        map_msg.msgissuerevision = 0
        map_msg.layertype = 0
        map_msg.layerid = 0

        intersection = IntersectionGeometry()
        ref_data = data["mapData"]["intersectionGeometry"]["referencePoint"]

        intersection.name = ref_data["descriptiveIntersctionName"]
        intersection.id = IntersectionReferenceID()
        intersection.id.intersectionid = int(ref_data["intersectionID"])
        intersection.id.roadregulatorid = 0

        intersection.revision = 1
        intersection.refpoint = Position3D()
        intersection.refpoint.latitude = int(ref_data["referenceLat"] * 1e7)
        intersection.refpoint.longitude = int(ref_data["referenceLon"] * 1e7)
        intersection.refpoint.elevation = int(float(ref_data["referenceElevation"]))
        intersection.lanewidth = int(ref_data["masterLaneWidth"])

        approaches = data["mapData"]["intersectionGeometry"]["laneList"]["approach"]
        lanes = []

        for approach in approaches:
            lane_type_key = "drivingLanes" if "drivingLanes" in approach else "crosswalkLanes"
            approach_type = approach["approachType"].lower()
            approach_id = int(approach["approachID"]) if approach["approachID"] != "-1" else 0

            for lane_data in approach[lane_type_key]:
                lane = GenericLane()
                lane.laneid = int(lane_data["laneID"])
                lane.name = lane_data["descriptiveName"]

                # Direction
                lane.ingressapproach = approach_id if approach_type == "ingress" else 0
                lane.egressapproach = approach_id if approach_type == "egress" else 0

                # Lane type
                lane.laneattributes = LaneAttributes()
                lane.laneattributes.lanetype = 1 if lane_data.get("laneType", "").lower() == "vehicle" else 0
                lane.laneattributes.directionaluse = ""
                lane.laneattributes.sharedwith = ""

                # Maneuvers
                lane.maneuvers = ",".join(map(str, lane_data.get("laneManeuvers", [])))

                # Nodes
                nodelist = NodeListXY()
                for node in lane_data["laneNodes"]:
                    node_xy = NodeXY()
                    node_xy.delta = NodeOffsetPointXY()
                    node_xy.delta.nodelatlon = Nodellmd64b()
                    node_xy.delta.nodelatlon.latitude = int(node["nodeLat"] * 1e7)
                    node_xy.delta.nodelatlon.longitude = int(node["nodeLong"] * 1e7)
                    nodelist.nodes.append(node_xy)
                lane.nodelist.append(nodelist)

                # === Add Connections ===
                for conn in lane_data.get("connections", []):
                    if conn.get("toLane") and conn.get("signal_id"):
                        connection = Connection()
                        connection.signalgroup = int(conn["signal_id"])

                        connection.connectinglane = ConnectingLane()
                        connection.connectinglane.laneid = int(conn["toLane"])
                        
                        # Optional: parse maneuvers
                        if "maneuvers" in conn:
                            maneuvers = conn["maneuvers"]
                            if 0 in maneuvers:
                                connection.connectinglane.maneuver.maneuverstraightallowed = 1
                            if 1 in maneuvers:
                                connection.connectinglane.maneuver.maneuverleftallowed = 1
                            if 2 in maneuvers:
                                connection.connectinglane.maneuver.maneuverrightallowed = 1
                            if 9 in maneuvers:
                                connection.connectinglane.maneuver.maneuveruturnallowed = 1

                        lane.connectsto.append(connection)

                lanes.append(lane)

        intersection.laneset = lanes
        map_msg.intersections.append(intersection)

        publisher_ = self.create_publisher(MAP, "MAP_Inspiration_1743", 10)
        self.MAP_publishers_.append((map_msg, publisher_))

    def MAPCallback(self):
        for map_msg, publisher_ in self.MAP_publishers_:
            publisher_.publish(map_msg)




# ---------------------------SPAT--------------------------------

    def SPATCallback(self):
        for signal_group_id, (traffic_signal, publisher_) in enumerate(self.SPAT_publishers_, start=1):
            spat = SPAT()

            # Timestamp
            moy, millis = self.getTimeStamp()
            spat.timestamp = moy
            spat.name = "ModularLabTest"

            # Intersection setup
            intersection = IntersectionState()
            intersection.name = "CARLA_INTERSECTION"
            intersection.id = IntersectionReferenceID()
            intersection.id.roadregulatorid = 0
            intersection.id.intersectionid = 39726  # Replace with real ID if needed
            intersection.revision = 1

            # Intersection status bits (you can customize further)
            status = IntersectionStatusObject()
            status.fixedtimeoperation = 1
            status.manualcontrolisenabled = 0
            status.signalpriorityisactive = 0
            intersection.status = status

            intersection.moy = moy
            intersection.timestamp = millis

            # Movement state per traffic light
            movement = MovementState()
            movement.movementname = f"signal_{signal_group_id}"
            movement.signalgroupid = signal_group_id

            move_event = MovementEvent()
            carla_state_name = traffic_signal.state.name  # E.g., 'Red', 'Yellow', 'Green'
            move_event.movementphasestate = CARLA_TO_J2735.get(carla_state_name, 0)

            now_ms = millis
            move_event.timing = TimeChangeDetails()
            move_event.timing.startime = now_ms
            move_event.timing.minendtime = now_ms + 8000
            move_event.timing.maxendtime = now_ms + 12000
            move_event.timing.likelytime = now_ms + 10000
            move_event.timing.confidence = 95
            move_event.timing.nexttime = 0

            movement.statetimespeed.append(move_event)
            intersection.states.append(movement)
            spat.intersections.append(intersection)

            # Publish
            publisher_.publish(spat)
# ----------------------------Set up a camera to watch over this luck intersection-------------------------
    def giveCameras(self):
        traffic_signals = self.world.get_actors().filter("traffic.traffic_light")
        counter = 1
        for traffic_signal in traffic_signals:
            publisher_=self.create_publisher(SPAT, "SPAT_TL_0"+str(counter), 10)
            self.SPAT_publishers_.append((traffic_signal, publisher_))
            # self.addRBGCameraSensor(traffic_signal, counter)
            counter += 1


    def RGBSensorcallback(self):
        # self.get_logger().info('Publishing: RGB Camera')
        while not self.image_queue.empty():
            image, camera_publisher_ = self.image_queue.get()
            ros_img = self.convertCARLAIMGtoROSIMG(image)
            camera_publisher_.publish(ros_img)
        
    def addRBGCameraSensor(self, traffic_signal, counter, x=-10,y=-2,z=10, pitch=-30,yaw=90,row=0):
        # create camera publisher
        camera_publisher_ = self.create_publisher(Image, f'traffic_signal_{counter}_camera', 1)
        # get camera blueprint
        camera_bp = self.blueprint_library.find('sensor.camera.rgb') 
        camera_location = carla.Location(x,y,z)
        camera_rotation = carla.Rotation(pitch,yaw,row)
        camera_transform = carla.Transform(camera_location,camera_rotation)
        camera_bp.set_attribute('image_size_x', '800')
        camera_bp.set_attribute('image_size_y', '600')
        camera_bp.set_attribute('sensor_tick', '0.1')
        # spawn onto infrastructure
        front_camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=traffic_signal)
        self.sensors.append(front_camera)
        front_camera.listen(lambda image: self.image_queue.put((image, camera_publisher_)))
    def destroy(self):
        for sensor in self.sensors:
            if sensor.is_alive:
                sensor.stop()
                sensor.destroy()
        return True
    def assistedLocalizationLaneID(self):
        # vehicles tell me where they are and what they look like
        # if they know where they are, then they should have their lane id already
        # if their gps isnt accurate, then RSU provide backup lane ids
        # then infrastructure tell them which lane they are in
        print("s")

def main():
    rclpy.init(args=None)

    # Things to do in this file
    # 1. Connect to world
    client = carla.Client('128.175.213.230', 2000)
    client.set_timeout(10.0) #seconds
    # 2. Get World Information
    world = client.get_world()
    # traffic_lights = world.get_actors().filter("traffic.traffic_light")

    spat = DefaultInfrastructure(world)
    

    try:
        rclpy.spin(spat)
    except KeyboardInterrupt:
        destroyed = spat.destroy()
        if destroyed:
            print("Infrastructure Destroyed")
    spat.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()