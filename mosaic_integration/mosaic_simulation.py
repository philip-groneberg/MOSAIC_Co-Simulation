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


# https://mosaic.dlr.de/docs/TraCI/Vehicle_Signalling.html
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


# https://mosaic.dlr.de/docs/Definition_of_Vehicles,_Vehicle_Types,_and_Routes.html#abstract_vehicle_class
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
            logging.info('Connection to grpx server. Host: %s Port: %s', host, port)
            stub = CarlaLink_pb2_grpc.CarlaLinkServiceStub(host + ":" + port)

        # Retrieving net from configuration file.
        self.net = _get_mosaic_net(cfg_file)

        # Variable to asign an id to new added actors.
        self._sequential_id = 0

        # Structures to keep track of the spawned and destroyed vehicles at each time step.
        self.spawned_actors = set()
        self.destroyed_actors = set()
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
        vclass = MosaicActorClass(vehicle.vclass)
        
        if vehicle.color is not None:
            color = (255, 255, 0, 100)
        else:
            color = vehicle.color

        length = float(vehicle.length)
        width = float(vehicle.width)
        height = float(vehicle.height)

        location = list((float(vehicle.location.x), float(vehicle.location.y), float(vehicle.location.z)))
        rotation = [float(vehicle.rotation.slope), float(vehicle.rotation.angle), 0.0]
        transform = carla.Transform(carla.Location(location[0], location[1], location[2]),
                                    carla.Rotation(rotation[0], rotation[1], rotation[2]))

        signals = vehicle.signals
        
        extent = carla.Vector3D(int(length) / 2.0, int(width) / 2.0, int(height) / 2.0)

        return MosaicActor(type_id, vclass, transform, signals, extent, color)

    def spawn_actor(self, type_id, color=None):
        """
        Spawns a new actor.

            :param type_id: vtype to be spawned.
            :param color: color attribute for this specific actor.
            :return: actor id if the actor is successfully spawned. Otherwise, INVALID_ACTOR_ID.
        """
        actor_id = 'carla' + str(self._sequential_id)

        self.step_result.add_actors.append(CarlaLink_pb2.SpawnRequest(actor_id = actor_id, route = 'carla_route', type_id = type_id, color = color))
        # add vehicle to grpc server so it can be processed by carla
        stub.AddVehicle(CarlaLink_pb2.Vehicle(id = actor_id, type_id = type_id, color = color))
        
        self._sequential_id += 1

        return actor_id

    def destroy_actor(self, actor_id):
        """
        Destroys the given actor.
        """
        self.step_result.remove_actors.append(CarlaLink_pb2.DestroyRequest(actor_id=actor_id))

    def synchronize_vehicle(self, vehicle_id, transform, signals=None):
        """
        Updates vehicle state.

            :param vehicle_id: id of the actor to be updated.
            :param transform: new vehicle transform (i.e., position and rotation).
            :param signals: new vehicle signals.
            :return: True if successfully updated. Otherwise, False.
        """
        loc_x, loc_y = transform.location.x, transform.location.y
        yaw = transform.rotation.yaw

        self.step_result.move_actors.append(CarlaLink_pb2.MoveRequest(actor_id=vehicle_id, loc_x=loc_x, loc_y=loc_y, yaw=yaw, keep_route=2, signals = signals))
        return True

    def get_sync_data():
        return self.step_result

    def tick(self):
        """
        Tick to mosaic simulation.
        """
        self.spawned_actors.clear()
        self.destroyed_actors.clear()

        del self.step_result.move_actors[:]
        del self.step_result.remove_actors[:]
        del self.step_result.add_actors[:]

        departed_actors = stub.GetDepartedIDList(CarlaLink_pb2.Empty())
        arrived_actors = stub.GetArrivedIDList(CarlaLink_pb2.Empty())

        for actor in departed_actors.actors:
            self.spawned_actors.add(actor.id)

        for actor in arrived_actors.actors:
            self.destroyed_actors.add(actor.id)

    @staticmethod
    def close():
        """
        Closes grpc client.
        """
        channel.close()
