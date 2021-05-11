#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Script to integrate CARLA and Eclipse-MOSAIC simulations
"""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import argparse
import logging
import time

from concurrent import futures
import grpc
import CarlaLink_pb2
import CarlaLink_pb2_grpc

# ==================================================================================================
# -- find carla module -----------------------------------------------------------------------------
# ==================================================================================================

import glob
import os
import sys

try:
    sys.path.append(
        glob.glob('../../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' %
                  (sys.version_info.major, sys.version_info.minor,
                   'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==================================================================================================
# -- find traci module -----------------------------------------------------------------------------
# ==================================================================================================

# if 'SUMO_HOME' in os.environ:
#     sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
# else:
#     sys.exit("please declare environment variable 'SUMO_HOME'")

# ==================================================================================================
# -- mosaic integration imports ----------------------------------------------------------------------
# ==================================================================================================

from mosaic_integration.bridge_helper import BridgeHelper  # pylint: disable=wrong-import-position
from mosaic_integration.carla_simulation import CarlaSimulation  # pylint: disable=wrong-import-position
from mosaic_integration.constants import INVALID_ACTOR_ID  # pylint: disable=wrong-import-position
from mosaic_integration.mosaic_simulation import MosaicSimulation  # pylint: disable=wrong-import-position

# ==================================================================================================
# -- synchronization_loop --------------------------------------------------------------------------
# ==================================================================================================


class SimulationSynchronization(object):
    """
    SimulationSynchronization class is responsible for the synchronization of mosaic and carla
    simulations.
    """
    def __init__(self,
                 mosaic_simulation,
                 carla_simulation,
                 tls_manager='none',
                 sync_vehicle_color=False,
                 sync_vehicle_lights=False):

        self.mosaic = mosaic_simulation
        self.carla = carla_simulation

        self.tls_manager = tls_manager
        self.sync_vehicle_color = sync_vehicle_color
        self.sync_vehicle_lights = sync_vehicle_lights

        if tls_manager == 'carla':
            self.mosaic.switch_off_traffic_lights()
        elif tls_manager == 'mosaic':
            self.carla.switch_off_traffic_lights()

        # Mapped actor ids.
        self.mosaic2carla_ids = {}  # Contains only actors controlled by mosaic.
        self.carla2mosaic_ids = {}  # Contains only actors controlled by carla.

        BridgeHelper.blueprint_library = self.carla.world.get_blueprint_library()
        BridgeHelper.offset = self.mosaic.get_net_offset()

        # Configuring carla simulation in sync mode.
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.carla.step_length
        self.carla.world.apply_settings(settings)

    def tick(self):
        """
        Tick to simulation synchronization
        """
        # -----------------
        # mosaic-->carla sync
        # -----------------
        self.mosaic.tick()

        # Spawning new mosaic actors in carla (i.e, not controlled by carla).
        mosaic_spawned_actors = self.mosaic.spawned_actors - set(self.carla2mosaic_ids.values())
        for mosaic_actor_id in mosaic_spawned_actors:
            self.mosaic.subscribe(mosaic_actor_id)
            mosaic_actor = self.mosaic.get_actor(mosaic_actor_id)

            carla_blueprint = BridgeHelper.get_carla_blueprint(mosaic_actor, self.sync_vehicle_color)
            if carla_blueprint is not None:
                carla_transform = BridgeHelper.get_carla_transform(mosaic_actor.transform,
                                                                   mosaic_actor.extent)

                carla_actor_id = self.carla.spawn_actor(carla_blueprint, carla_transform)
                if carla_actor_id != INVALID_ACTOR_ID:
                    self.mosaic2carla_ids[mosaic_actor_id] = carla_actor_id
            else:
                self.mosaic.unsubscribe(mosaic_actor_id)

        # Destroying mosaic arrived actors in carla.
        for mosaic_actor_id in self.mosaic.destroyed_actors:
            if mosaic_actor_id in self.mosaic2carla_ids:
                self.carla.destroy_actor(self.mosaic2carla_ids.pop(mosaic_actor_id))

        # Updating mosaic actors in carla.
        for mosaic_actor_id in self.mosaic2carla_ids:
            carla_actor_id = self.mosaic2carla_ids[mosaic_actor_id]

            mosaic_actor = self.mosaic.get_actor(mosaic_actor_id)
            carla_actor = self.carla.get_actor(carla_actor_id)

            carla_transform = BridgeHelper.get_carla_transform(mosaic_actor.transform,
                                                               mosaic_actor.extent)
            if self.sync_vehicle_lights:
                carla_lights = BridgeHelper.get_carla_lights_state(carla_actor.get_light_state(),
                                                                   mosaic_actor.signals)
            else:
                carla_lights = None

            self.carla.synchronize_vehicle(carla_actor_id, carla_transform, carla_lights)

        # Updates traffic lights in carla based on mosaic information.
        if self.tls_manager == 'mosaic':
            common_landmarks = self.mosaic.traffic_light_ids & self.carla.traffic_light_ids
            for landmark_id in common_landmarks:
                mosaic_tl_state = self.mosaic.get_traffic_light_state(landmark_id)
                carla_tl_state = BridgeHelper.get_carla_traffic_light_state(mosaic_tl_state)

                self.carla.synchronize_traffic_light(landmark_id, carla_tl_state)

        # -----------------
        # carla-->mosaic sync
        # -----------------
        self.carla.tick()

        # Spawning new carla actors (not controlled by mosaic)
        carla_spawned_actors = self.carla.spawned_actors - set(self.mosaic2carla_ids.values())
        for carla_actor_id in carla_spawned_actors:
            carla_actor = self.carla.get_actor(carla_actor_id)

            type_id = BridgeHelper.get_mosaic_vtype(carla_actor)
            color = carla_actor.attributes.get('color', None) if self.sync_vehicle_color else None
            if type_id is not None:
                mosaic_actor_id = self.mosaic.spawn_actor(type_id, color)
                if mosaic_actor_id != INVALID_ACTOR_ID:
                    self.carla2mosaic_ids[carla_actor_id] = mosaic_actor_id
                    self.mosaic.subscribe(mosaic_actor_id)

        # Destroying required carla actors in mosaic.
        for carla_actor_id in self.carla.destroyed_actors:
            if carla_actor_id in self.carla2mosaic_ids:
                self.mosaic.destroy_actor(self.carla2mosaic_ids.pop(carla_actor_id))

        # Updating carla actors in mosaic.
        for carla_actor_id in self.carla2mosaic_ids:
            mosaic_actor_id = self.carla2mosaic_ids[carla_actor_id]

            carla_actor = self.carla.get_actor(carla_actor_id)
            mosaic_actor = self.mosaic.get_actor(mosaic_actor_id)

            mosaic_transform = BridgeHelper.get_mosaic_transform(carla_actor.get_transform(),
                                                             carla_actor.bounding_box.extent)
            if self.sync_vehicle_lights:
                carla_lights = self.carla.get_actor_light_state(carla_actor_id)
                if carla_lights is not None:
                    mosaic_lights = BridgeHelper.get_mosaic_lights_state(mosaic_actor.signals,
                                                                     carla_lights)
                else:
                    mosaic_lights = None
            else:
                mosaic_lights = None

            self.mosaic.synchronize_vehicle(mosaic_actor_id, mosaic_transform, mosaic_lights)

        # Updates traffic lights in mosaic based on carla information.
        if self.tls_manager == 'carla':
            common_landmarks = self.mosaic.traffic_light_ids & self.carla.traffic_light_ids
            for landmark_id in common_landmarks:
                carla_tl_state = self.carla.get_traffic_light_state(landmark_id)
                mosaic_tl_state = BridgeHelper.get_mosaic_traffic_light_state(carla_tl_state)

                # Updates all the mosaic links related to this landmark.
                self.mosaic.synchronize_traffic_light(landmark_id, mosaic_tl_state)

        return self.mosaic.step_result

    def close(self):
        """
        Cleans synchronization.
        """
        # Configuring carla simulation in async mode.
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.carla.world.apply_settings(settings)

        # Destroying synchronized actors.
        for carla_actor_id in self.mosaic2carla_ids.values():
            self.carla.destroy_actor(carla_actor_id)

        for mosaic_actor_id in self.carla2mosaic_ids.values():
            self.mosaic.destroy_actor(mosaic_actor_id)

        # Closing mosaic and carla client.
        self.carla.close()
        self.mosaic.close()

class CarlaLinkServiceServicer(CarlaLink_pb2_grpc.CarlaLinkServiceServicer, object):
    """Provides methods that implement functionality of route guide server."""

    def __init__(self, object):
      self.sync = object
      self.vehicles = dict()
      self.spawned_actors = list()
      self.destroyed_actors = list()

    def SimulationStep(self, request, context):
      # logging.debug("SimulationStep call recieved! Time: ", time.time())
      step_result = self.sync.tick()

      for actor in self.destroyed_actors:
        self.vehicles.pop(actor.id)

      del self.destroyed_actors[:]
      del self.spawned_actors[:]

      return step_result

    def GetActor(self, request, context):
      # logging.debug('GetActor call recieved!')
      return self.vehicles[request.actor_id]

    def GetDepartedIDList(self, request, context):
      # logging.debug('GetDepartedIDList call recieved!')
      departed_actors = CarlaLink_pb2.DepartedActors()
      for actor in self.spawned_actors:
        departed_actors.actors.append(actor)
      return departed_actors

    def GetArrivedIDList(self, request, context):
      # logging.debug('GetArrivedIDList call recieved!')
      arrived_actors = CarlaLink_pb2.ArrivedActors()
      for actor in self.destroyed_actors:
        arrived_actors.actors.append(actor)
      return arrived_actors

    def AddVehicle(self, request, context):
      # logging.debug('AddVehicle call recieved! id:', request.id)
      self.spawned_actors.append(request)
      self.vehicles.update({request.id: request})
      return CarlaLink_pb2.Empty()

    def RemoveVehicle(self, request, context):
      # logging.debug('RemoveVehicle call recieved! id:', request.id)
      self.destroyed_actors.append(request)
      return CarlaLink_pb2.Empty()

    def UpdateVehicle(self, request, context):
      # logging.debug('UpdateVehicle call recieved! id:', request.id)
      self.vehicles.update({request.id: request})
      return CarlaLink_pb2.Empty()

def synchronization_loop(args):
    """
    Entry point for mosaic-carla co-simulation.
    """
    mosaic_simulation = MosaicSimulation(args.mosaic_cfg_file, args.step_length, args.mosaic_host,
                                     args.mosaic_port, args.mosaic_gui, args.client_order)
    carla_simulation = CarlaSimulation(args.carla_host, args.carla_port, args.step_length)

    synchronization = SimulationSynchronization(mosaic_simulation, carla_simulation, args.tls_manager,
                                                args.sync_vehicle_color, args.sync_vehicle_lights)
    try:
        logging.info('Starting grpc server on port 50051')
        server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        CarlaLink_pb2_grpc.add_CarlaLinkServiceServicer_to_server(
            CarlaLinkServiceServicer(synchronization), server)
        server.add_insecure_port('[::]:50051')
        server.start()
        logging.info('Waiting for incomming calls...')
        server.wait_for_termination()

    except KeyboardInterrupt:
        logging.info('Cancelled by user.')

    finally:
        logging.info('Cleaning synchronization')

        synchronization.close()


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('mosaic_cfg_file', type=str, help='mosaic configuration file')
    argparser.add_argument('--carla-host',
                           metavar='H',
                           default='127.0.0.1',
                           help='IP of the carla host server (default: 127.0.0.1)')
    argparser.add_argument('--carla-port',
                           metavar='P',
                           default=2000,
                           type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--mosaic-host',
                           metavar='H',
                           default=None,
                           help='IP of the mosaic host server (default: 127.0.0.1)')
    argparser.add_argument('--mosaic-port',
                           metavar='P',
                           default=None,
                           type=int,
                           help='TCP port to listen to (default: 8813)')
    argparser.add_argument('--mosaic-gui', action='store_true', help='run the gui version of mosaic')
    argparser.add_argument('--step-length',
                           default=0.05,
                           type=float,
                           help='set fixed delta seconds (default: 0.05s)')
    argparser.add_argument('--client-order',
                           metavar='TRACI_CLIENT_ORDER',
                           default=1,
                           type=int,
                           help='client order number for the co-simulation TraCI connection (default: 1)')
    argparser.add_argument('--sync-vehicle-lights',
                           action='store_true',
                           help='synchronize vehicle lights state (default: False)')
    argparser.add_argument('--sync-vehicle-color',
                           action='store_true',
                           help='synchronize vehicle color (default: False)')
    argparser.add_argument('--sync-vehicle-all',
                           action='store_true',
                           help='synchronize all vehicle properties (default: False)')
    argparser.add_argument('--tls-manager',
                           type=str,
                           choices=['none', 'mosaic', 'carla'],
                           help="select traffic light manager (default: none)",
                           default='none')
    argparser.add_argument('--debug', action='store_true', help='enable debug messages')
    arguments = argparser.parse_args()

    if arguments.sync_vehicle_all is True:
        arguments.sync_vehicle_lights = True
        arguments.sync_vehicle_color = True

    if arguments.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    synchronization_loop(arguments)
