#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""Spawn Mosaic NPCs vehicles into the simulation"""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import argparse
import json
import logging
import random
import re
import shutil
import tempfile
import time

import lxml.etree as ET  # pylint: disable=wrong-import-position

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

if 'MOSAIC_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['MOSAIC_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'MOSAIC_HOME'")

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import mosaiclib  # pylint: disable=wrong-import-position
import traci  # pylint: disable=wrong-import-position

from mosaic_integration.carla_simulation import CarlaSimulation  # pylint: disable=wrong-import-position
from mosaic_integration.mosaic_simulation import MosaicSimulation  # pylint: disable=wrong-import-position

from run_synchronization import SimulationSynchronization  # pylint: disable=wrong-import-position

from util.netconvert_carla import netconvert_carla

# ==================================================================================================
# -- main ------------------------------------------------------------------------------------------
# ==================================================================================================


def write_mosaiccfg_xml(cfg_file, net_file, vtypes_file, viewsettings_file, additional_traci_clients=0):
    """
    Writes mosaic configuration xml file.
    """
    root = ET.Element('configuration')

    input_tag = ET.SubElement(root, 'input')
    ET.SubElement(input_tag, 'net-file', {'value': net_file})
    ET.SubElement(input_tag, 'route-files', {'value': vtypes_file})

    gui_tag = ET.SubElement(root, 'gui_only')
    ET.SubElement(gui_tag, 'gui-settings-file', {'value': viewsettings_file})

    ET.SubElement(root, 'num-clients', {'value': str(additional_traci_clients+1)})

    tree = ET.ElementTree(root)
    tree.write(cfg_file, pretty_print=True, encoding='UTF-8', xml_declaration=True)


def main(args):

    # Temporal folder to save intermediate files.
    tmpdir = tempfile.mkdtemp()

    # ----------------
    # carla simulation
    # ----------------
    carla_simulation = CarlaSimulation(args.host, args.port, args.step_length)

    world = carla_simulation.client.get_world()
    current_map = world.get_map()

    xodr_file = os.path.join(tmpdir, current_map.name + '.xodr')
    current_map.save_to_disk(xodr_file)

    # ---------------
    # mosaic simulation
    # ---------------
    net_file = os.path.join(tmpdir, current_map.name + '.net.xml')
    netconvert_carla(xodr_file, net_file, guess_tls=True)

    basedir = os.path.dirname(os.path.realpath(__file__))
    cfg_file = os.path.join(tmpdir, current_map.name + '.mosaiccfg')
    vtypes_file = os.path.join(basedir, 'examples', 'carlavtypes.rou.xml')
    viewsettings_file = os.path.join(basedir, 'examples', 'viewsettings.xml')
    write_mosaiccfg_xml(cfg_file, net_file, vtypes_file, viewsettings_file, args.additional_traci_clients)

    mosaic_net = mosaiclib.net.readNet(net_file)
    mosaic_simulation = MosaicSimulation(cfg_file,
                                     args.step_length,
                                     host=args.mosaic_host,
                                     port=args.mosaic_port,
                                     mosaic_gui=args.mosaic_gui,
                                     client_order=args.client_order)

    # ---------------
    # synchronization
    # ---------------
    synchronization = SimulationSynchronization(mosaic_simulation, carla_simulation, args.tls_manager,
                                                args.sync_vehicle_color, args.sync_vehicle_lights)

    try:
        # ----------
        # Blueprints
        # ----------
        with open('data/vtypes.json') as f:
            vtypes = json.load(f)['carla_blueprints']

        blueprints = vtypes.keys()

        filterv = re.compile(args.filterv)
        blueprints = list(filter(filterv.search, blueprints))

        if args.safe:
            blueprints = [
                x for x in blueprints if vtypes[x]['vClass'] not in ('motorcycle', 'bicycle')
            ]
            blueprints = [x for x in blueprints if not x.endswith('isetta')]
            blueprints = [x for x in blueprints if not x.endswith('carlacola')]
            blueprints = [x for x in blueprints if not x.endswith('cybertruck')]
            blueprints = [x for x in blueprints if not x.endswith('t2')]

        if not blueprints:
            raise RuntimeError('No blueprints available due to user restrictions.')

        if args.number_of_walkers > 0:
            logging.warning('Pedestrians are not supported yet. No walkers will be spawned.')

        # --------------
        # Spawn vehicles
        # --------------
        # Spawns mosaic NPC vehicles.
        mosaic_edges = mosaic_net.getEdges()

        for i in range(args.number_of_vehicles):
            type_id = random.choice(blueprints)
            vclass = vtypes[type_id]['vClass']

            allowed_edges = [e for e in mosaic_edges if e.allows(vclass)]
            edge = random.choice(allowed_edges)

            traci.route.add('route_{}'.format(i), [edge.getID()])
            traci.vehicle.add('mosaic_{}'.format(i), 'route_{}'.format(i), typeID=type_id)

        while True:
            start = time.time()

            synchronization.tick()

            # Updates vehicle routes
            for vehicle_id in traci.vehicle.getIDList():
                route = traci.vehicle.getRoute(vehicle_id)
                index = traci.vehicle.getRouteIndex(vehicle_id)
                vclass = traci.vehicle.getVehicleClass(vehicle_id)

                if index == (len(route) - 1):
                    current_edge = mosaic_net.getEdge(route[index])
                    available_edges = list(current_edge.getAllowedOutgoing(vclass).keys())
                    if available_edges:
                        next_edge = random.choice(available_edges)

                        new_route = [current_edge.getID(), next_edge.getID()]
                        traci.vehicle.setRoute(vehicle_id, new_route)

            end = time.time()
            elapsed = end - start
            if elapsed < args.step_length:
                time.sleep(args.step_length - elapsed)

    except KeyboardInterrupt:
        logging.info('Cancelled by user.')

    finally:
        synchronization.close()

        if os.path.exists(tmpdir):
            shutil.rmtree(tmpdir)



if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--host',
                           metavar='H',
                           default='127.0.0.1',
                           help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('-p',
                           '--port',
                           metavar='P',
                           default=2000,
                           type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--mosaic-host',
                           default=None,
                           help='IP of the mosaic host server (default: None)')
    argparser.add_argument('--mosaic-port',
                           default=None,
                           type=int,
                           help='TCP port to listen to (default: None)')
    argparser.add_argument('-n',
                           '--number-of-vehicles',
                           metavar='N',
                           default=10,
                           type=int,
                           help='number of vehicles (default: 10)')
    argparser.add_argument('-w',
                           '--number-of-walkers',
                           metavar='W',
                           default=0,
                           type=int,
                           help='number of walkers (default: 0)')
    argparser.add_argument('--safe',
                           action='store_true',
                           help='avoid spawning vehicles prone to accidents')
    argparser.add_argument('--filterv',
                           metavar='PATTERN',
                           default='vehicle.*',
                           help='vehicles filter (default: "vehicle.*")')
    argparser.add_argument('--filterw',
                           metavar='PATTERN',
                           default='walker.pedestrian.*',
                           help='pedestrians filter (default: "walker.pedestrian.*")')
    argparser.add_argument('--mosaic-gui', action='store_true', help='run the gui version of mosaic')
    argparser.add_argument('--step-length',
                           default=0.05,
                           type=float,
                           help='set fixed delta seconds (default: 0.05s)')
    argparser.add_argument('--additional-traci-clients',
                           metavar='TRACI_CLIENTS',
                           default=0,
                           type=int,
                           help='number of additional TraCI clients to wait for (default: 0)')
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
    args = argparser.parse_args()

    if args.sync_vehicle_all is True:
        args.sync_vehicle_lights = True
        args.sync_vehicle_color = True

    if args.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    main(args)
