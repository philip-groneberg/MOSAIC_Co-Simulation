#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
""" This module is responsible for the management of the mosaic simulation. """

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import collections
import enum
import logging
import os
import numpy as np

import carla  # pylint: disable=import-error

import os, sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:   
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib  # pylint: disable=import-error
import traci  # pylint: disable=import-error

import grpc

import CarlaLink_pb2
import CarlaLink_pb2_grpc

from .constants import INVALID_ACTOR_ID

import lxml.etree as ET  # pylint: disable=import-error

# ==================================================================================================
# -- mosaic definitions ------------------------------------------------------------------------------
# ==================================================================================================


# https://sumo.dlr.de/docs/Simulation/Traffic_Lights.html#signal_state_definitions
class MosaicSignalState(object):
    """
    MosaicSignalState contains the different traffic light states.
    """
    RED = 'r'
    YELLOW = 'y'
    GREEN = 'G'
    GREEN_WITHOUT_PRIORITY = 'g'
    GREEN_RIGHT_TURN = 's'
    RED_YELLOW = 'u'
    OFF_BLINKING = 'o'
    OFF = 'O'


# https://sumo.dlr.de/docs/TraCI/Vehicle_Signalling.html
class MosaicVehSignal(object):
    """
    MosaicVehSignal contains the different mosaic vehicle signals.
    """
    BLINKER_RIGHT = 1 << 0
    BLINKER_LEFT = 1 << 1
    BLINKER_EMERGENCY = 1 << 2
    BRAKELIGHT = 1 << 3
    FRONTLIGHT = 1 << 4
    FOGLIGHT = 1 << 5
    HIGHBEAM = 1 << 6
    BACKDRIVE = 1 << 7
    WIPER = 1 << 8
    DOOR_OPEN_LEFT = 1 << 9
    DOOR_OPEN_RIGHT = 1 << 10
    EMERGENCY_BLUE = 1 << 11
    EMERGENCY_RED = 1 << 12
    EMERGENCY_YELLOW = 1 << 13


# https://sumo.dlr.de/docs/Definition_of_Vehicles,_Vehicle_Types,_and_Routes.html#abstract_vehicle_class
class MosaicActorClass(enum.Enum):
    """
    MosaicActorClass enumerates the different mosaic actor classes.
    """
    IGNORING = "ignoring"
    PRIVATE = "private"
    EMERGENCY = "emergency"
    AUTHORITY = "authority"
    ARMY = "army"
    VIP = "vip"
    PEDESTRIAN = "pedestrian"
    PASSENGER = "passenger"
    HOV = "hov"
    TAXI = "taxi"
    BUS = "bus"
    COACH = "coach"
    DELIVERY = "delivery"
    TRUCK = "truck"
    TRAILER = "trailer"
    MOTORCYCLE = "motorcycle"
    MOPED = "moped"
    BICYCLE = "bicycle"
    EVEHICLE = "evehicle"
    TRAM = "tram"
    RAIL_URBAN = "rail_urban"
    RAIL = "rail"
    RAIL_ELECTRIC = "rail_electric"
    RAIL_FAST = "rail_fast"
    SHIP = "ship"
    CUSTOM1 = "custom1"
    CUSTOM2 = "custom2"


MosaicActor = collections.namedtuple('MosaicActor', 'type_id vclass transform signals extent color')

# ==================================================================================================
# -- mosaic simulation -------------------------------------------------------------------------------
# ==================================================================================================

def _get_mosaic_net(cfg_file):
    """
    Returns mosaic net.

    This method reads the mosaic configuration file and retrieve the mosaic net filename to create the
    net.
    """
    cfg_file = os.path.join(os.getcwd(), cfg_file)

    tree = ET.parse(cfg_file)
    tag = tree.find('//net-file')
    if tag is None:
        return None

    net_file = os.path.join(os.path.dirname(cfg_file), tag.get('value'))
    logging.debug('Reading net file: %s', net_file)

    mosaic_net = traci.sumolib.net.readNet(net_file)
    return mosaic_net

class MosaicSimulation(object):
    """
    MosaicSimulation is responsible for the management of the mosaic simulation.
    """
    def __init__(self, cfg_file, step_length, host=None, port=None, mosaic_gui=False, client_order=1):
        # if mosaic_gui is True:
        #     sumo_binary = sumolib.checkBinary('sumo-gui')
        # else:
        #     sumo_binary = sumolib.checkBinary('sumo')

        global channel
        global stub
        
        if host is None or port is None:
            logging.info('Connect to grpc server on port 50051')
            channel = grpc.insecure_channel('localhost:50051')
            stub = CarlaLink_pb2_grpc.CarlaLinkServiceStub(channel)

        else:
            logging.info('Connection to grpc server. Host: %s Port: %s', host, port)
            stub = CarlaLink_pb2_grpc.CarlaLinkServiceStub(host + ":" + port)

        # Retrieving net from configuration file.
        self.net = _get_mosaic_net(cfg_file)

        # Variable to assign an id to new added actors.
        self._sequential_id = 0

        # Structures to keep track of the spawned and destroyed vehicles at each time step.
        self.spawned_actors = set()
        self.destroyed_actors = set()
        self.traffic_light_ids = set()
        self.step_result = CarlaLink_pb2.StepResult()

    @staticmethod
    def subscribe(actor_id):
        """
        Subscribe the given actor to the following variables:
        [Is kept to make future implementations and updates easier, but can be removed]

            * Type.
            * Vehicle class.
            * Color.
            * Length, Width, Height.
            * Position3D (i.e., x, y, z).
            * Angle, Slope.
            * Speed.
            * Lateral speed.
            * Signals.
        """

    @staticmethod
    def unsubscribe(actor_id):
        """
        Unsubscribe the given actor from receiving updated information each step.
        [Is kept to make future implementations and updates easier, but can be removed]
        """

    def get_net_offset(self):
        """
        Accessor for mosaic net offset.
        """
        if self.net is None:
            return (0, 0)
        return self.net.getLocationOffset()

    @staticmethod
    def get_actor(actor_id):
        """
        Accessor for mosaic actor.
        """
        vehicle = stub.GetActor(CarlaLink_pb2.ActorRequest(actor_id = actor_id))
        
        type_id = vehicle.type_id

        if vehicle.vclass:
            vclass = MosaicActorClass(vehicle.vclass)
        else:
            # logging.info("get_actor: Missing vclass for '%s', using 'passenger' instead!", actor_id)
            vclass = MosaicActorClass("passenger")
        
        if vehicle.color is not None:
            try:
                color = list(map(int, vehicle.color.split(",")))
            except ValueError:
                color = (255, 255, 0, 100)
        else:
            color = (255, 255, 0, 100)

        if vehicle.length and vehicle.width and vehicle.height:
            length = float(vehicle.length)
            width = float(vehicle.width)
            height = float(vehicle.height)
        else:
            # logging.info("get_actor: Missing dimension for '%s' (%s,%s,%s), using base values (3.97,1.86,1.62) instead!", actor_id, vehicle.length, vehicle.width, vehicle.height)
            length = 3.97
            width = 1.86
            height = 1.62

        location = list((float(vehicle.location.x), float(vehicle.location.y), float(vehicle.location.z)))
        rotation = [float(vehicle.rotation.slope), float(vehicle.rotation.angle), 0.0]
        transform = carla.Transform(carla.Location(location[0], location[1], location[2]),
                                    carla.Rotation(rotation[0], rotation[1], rotation[2]))

        signals = vehicle.signals
        
        extent = carla.Vector3D(float(length) / 2.0, float(width) / 2.0, float(height) / 2.0)

        return MosaicActor(type_id, vclass, transform, signals, extent, color)

    def spawn_actor(self, type_id, class_id, color=None):
        """
        Spawns a new actor.

            :param type_id: vtype to be spawned.
            :param class_id: vehicle class to be spawned.
            :param color: color attribute for this specific actor.
            :return: actor id if the actor is successfully spawned. Otherwise, INVALID_ACTOR_ID.
        """
        actor_id = 'carla' + str(self._sequential_id)

        self.step_result.add_actors.append(CarlaLink_pb2.SpawnRequest(actor_id=actor_id, route='carla_route',
                                                                      type_id=type_id, color=color, length=3.97,
                                                                      width=1.86, height=1.62, class_id=class_id))
        # add vehicle to grpc server so it can be processed by carla
        stub.AddVehicle(CarlaLink_pb2.Vehicle(id = actor_id, type_id = type_id, color = color))
        
        self._sequential_id += 1

        return actor_id

    def destroy_actor(self, actor_id):
        """
        Destroys the given actor.
        """
        self.step_result.remove_actors.append(CarlaLink_pb2.DestroyRequest(actor_id=actor_id))

    def get_traffic_light_state(self, landmark_id):
        """
        Accessor for traffic light state.

        If the traffic light does not exist, returns None.
        """
        # return self.traffic_light_manager.get_state(landmark_id)
        traffic_light = stub.GetTrafficLight(CarlaLink_pb2.LandmarkRequest(landmark_id = landmark_id))
        return traffic_light.state

    def switch_off_traffic_lights(self):
        """
        Switch off all traffic lights.
        """
        # self.traffic_light_manager.switch_off()
        # maybe switch off traffic lights in Mosaic (if possible)
        return

    def synchronize_vehicle(self, vehicle_id, transform, signals=None):
        """
        Updates vehicle state.

            :param vehicle_id: id of the actor to be updated.
            :param transform: new vehicle transform (i.e., position and rotation).
            :param signals: new vehicle signals.
            :return: True if successfully updated. Otherwise, False.
        """
        loc_x, loc_y, loc_z = transform.location.x, transform.location.y, transform.location.z
        yaw, slope = transform.rotation.yaw, transform.rotation.pitch

        self.step_result.move_actors.append(CarlaLink_pb2.MoveRequest(actor_id=vehicle_id, loc_x=loc_x, loc_y=loc_y,
                                                                      loc_z=loc_z, yaw=yaw, slope=slope, keep_route=2,
                                                                      signals=signals))
        return True

    def get_sync_data():
        return self.step_result

    def synchronize_traffic_light(self, landmark_id, state):
        """
        Updates traffic light state.

            :param tl_id: id of the traffic light to be updated (logic id, link index).
            :param state: new traffic light state.
            :return: True if successfully updated. Otherwise, False.
        """
        # logging.debug("Mosaic sync TL: %s with state: %s", landmark_id, state)
        self.step_result.traffic_light_updates.append(CarlaLink_pb2.TrafficLight(landmark_id = landmark_id, state = state))

    def process_lidar(self, data, sensor_id):
        """
        Transfer of LIDAR sensor data to the stepResult that get transferred to Mosaic
        :param data: LIDAR data
        :param sensor_id: ID of the vehicle the sensor is attached to
        :return:
        """

        print("Create sensor data for sensor:", sensor_id, " at ", str(data.timestamp))

        # code taken from lidar_to_camera.py example by Carla
        # Get the lidar data and convert it to a numpy array.
        p_cloud_size = len(data)
        p_cloud = np.copy(np.frombuffer(data.raw_data, dtype=np.dtype('f4')))
        p_cloud = np.reshape(p_cloud, (p_cloud_size, 4))

        # Lidar intensity array of shape (p_cloud_size,) but, for now, let's
        # focus on the 3D points.
        intensity = np.array(p_cloud[:, 3])

        # Point cloud in lidar sensor space array of shape (3, p_cloud_size).
        local_lidar_points = np.array(p_cloud[:, :3]).T

        # Add an extra 1.0 at the end of each 3d point so it becomes of
        # shape (4, p_cloud_size) and it can be multiplied by a (4, 4) matrix.
        local_lidar_points = np.r_[local_lidar_points, [np.ones(local_lidar_points.shape[1])]]

        # This (4, 4) matrix transforms the points from lidar space to world space.
        # lidar_2_world = self.sensor.get_transform().get_matrix()
        lidar_2_world = data.transform.get_matrix()
        # Transform the points from lidar space to world space.
        world_points = np.dot(lidar_2_world, local_lidar_points)

        # apply offset to Mosaic
        offset = [[self.get_net_offset()[0]], [-self.get_net_offset()[1]], [0], [0]]
        world_points_with_offset = world_points + offset
        # mirror y axis
        world_points_with_offset *= [[1], [-1], [1], [1]]
        # lose unnecessary 4th row
        world_points_with_offset = world_points_with_offset[:3, :]

        sensor_location = CarlaLink_pb2.Location(x = float(data.transform.location.x), y = float(data.transform.location.y), z = float(data.transform.location.z))
        sensor_data = CarlaLink_pb2.SensorData(id = sensor_id, timestamp = str(data.timestamp), minRange = 0, maxRange = 300, location = sensor_location)

        for i, intensity_value in enumerate(intensity):
            # print('Intensity:', intensity_value)
            if intensity_value > 0:
                lidar_point = CarlaLink_pb2.Location(x = float(world_points_with_offset[0][i]), y = float(world_points_with_offset[1][i]), z = float(world_points_with_offset[2][i]))
                # print('Recorded LIDAR point:', lidar_point)
                sensor_data.lidar_points.append(lidar_point)

        self.step_result.sensor_data.append(sensor_data)

    def tick(self):
        """
        Tick to mosaic simulation.
        """
        print("mosaic tick called!")
        self.spawned_actors.clear()
        self.destroyed_actors.clear()
        self.traffic_light_ids.clear()

        del self.step_result.move_actors[:]
        del self.step_result.remove_actors[:]
        del self.step_result.add_actors[:]
        del self.step_result.traffic_light_updates[:]
        del self.step_result.sensor_data[:]

        departed_actors = stub.GetDepartedIDList(CarlaLink_pb2.Empty())
        arrived_actors = stub.GetArrivedIDList(CarlaLink_pb2.Empty())
        traffic_lights = stub.GetTrafficLightIDList(CarlaLink_pb2.Empty())

        for actor in departed_actors.actors:
            self.spawned_actors.add(actor.id)

        for actor in arrived_actors.actors:
            self.destroyed_actors.add(actor.id)

        for traffic_light in traffic_lights.traffic_lights:
            self.traffic_light_ids.add(traffic_light.landmark_id)


    @staticmethod
    def close():
        """
        Closes grpc client.
        """
        channel.close()
