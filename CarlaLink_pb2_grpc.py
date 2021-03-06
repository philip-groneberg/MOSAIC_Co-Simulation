# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc

import CarlaLink_pb2 as CarlaLink__pb2


class CarlaLinkServiceStub(object):
    """The greeting service definition.
    """

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.GetActor = channel.unary_unary(
                '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/GetActor',
                request_serializer=CarlaLink__pb2.ActorRequest.SerializeToString,
                response_deserializer=CarlaLink__pb2.Vehicle.FromString,
                )
        self.GetDepartedIDList = channel.unary_unary(
                '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/GetDepartedIDList',
                request_serializer=CarlaLink__pb2.Empty.SerializeToString,
                response_deserializer=CarlaLink__pb2.DepartedActors.FromString,
                )
        self.GetArrivedIDList = channel.unary_unary(
                '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/GetArrivedIDList',
                request_serializer=CarlaLink__pb2.Empty.SerializeToString,
                response_deserializer=CarlaLink__pb2.ArrivedActors.FromString,
                )
        self.AddVehicle = channel.unary_unary(
                '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/AddVehicle',
                request_serializer=CarlaLink__pb2.Vehicle.SerializeToString,
                response_deserializer=CarlaLink__pb2.Empty.FromString,
                )
        self.RemoveVehicle = channel.unary_unary(
                '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/RemoveVehicle',
                request_serializer=CarlaLink__pb2.Vehicle.SerializeToString,
                response_deserializer=CarlaLink__pb2.Empty.FromString,
                )
        self.UpdateVehicle = channel.unary_unary(
                '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/UpdateVehicle',
                request_serializer=CarlaLink__pb2.Vehicle.SerializeToString,
                response_deserializer=CarlaLink__pb2.Empty.FromString,
                )
        self.SimulationStep = channel.unary_unary(
                '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/SimulationStep',
                request_serializer=CarlaLink__pb2.Step.SerializeToString,
                response_deserializer=CarlaLink__pb2.StepResult.FromString,
                )
        self.GetTrafficLight = channel.unary_unary(
                '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/GetTrafficLight',
                request_serializer=CarlaLink__pb2.LandmarkRequest.SerializeToString,
                response_deserializer=CarlaLink__pb2.TrafficLight.FromString,
                )
        self.GetTrafficLightIDList = channel.unary_unary(
                '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/GetTrafficLightIDList',
                request_serializer=CarlaLink__pb2.Empty.SerializeToString,
                response_deserializer=CarlaLink__pb2.TrafficLights.FromString,
                )
        self.UpdateTrafficLight = channel.unary_unary(
                '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/UpdateTrafficLight',
                request_serializer=CarlaLink__pb2.TrafficLight.SerializeToString,
                response_deserializer=CarlaLink__pb2.Empty.FromString,
                )
        self.AddSensor = channel.unary_unary(
                '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/AddSensor',
                request_serializer=CarlaLink__pb2.Sensor.SerializeToString,
                response_deserializer=CarlaLink__pb2.Sensor.FromString,
                )
        self.RemoveSensor = channel.unary_unary(
                '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/RemoveSensor',
                request_serializer=CarlaLink__pb2.Sensor.SerializeToString,
                response_deserializer=CarlaLink__pb2.Empty.FromString,
                )


class CarlaLinkServiceServicer(object):
    """The greeting service definition.
    """

    def GetActor(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetDepartedIDList(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetArrivedIDList(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def AddVehicle(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def RemoveVehicle(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def UpdateVehicle(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def SimulationStep(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetTrafficLight(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetTrafficLightIDList(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def UpdateTrafficLight(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def AddSensor(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def RemoveSensor(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_CarlaLinkServiceServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'GetActor': grpc.unary_unary_rpc_method_handler(
                    servicer.GetActor,
                    request_deserializer=CarlaLink__pb2.ActorRequest.FromString,
                    response_serializer=CarlaLink__pb2.Vehicle.SerializeToString,
            ),
            'GetDepartedIDList': grpc.unary_unary_rpc_method_handler(
                    servicer.GetDepartedIDList,
                    request_deserializer=CarlaLink__pb2.Empty.FromString,
                    response_serializer=CarlaLink__pb2.DepartedActors.SerializeToString,
            ),
            'GetArrivedIDList': grpc.unary_unary_rpc_method_handler(
                    servicer.GetArrivedIDList,
                    request_deserializer=CarlaLink__pb2.Empty.FromString,
                    response_serializer=CarlaLink__pb2.ArrivedActors.SerializeToString,
            ),
            'AddVehicle': grpc.unary_unary_rpc_method_handler(
                    servicer.AddVehicle,
                    request_deserializer=CarlaLink__pb2.Vehicle.FromString,
                    response_serializer=CarlaLink__pb2.Empty.SerializeToString,
            ),
            'RemoveVehicle': grpc.unary_unary_rpc_method_handler(
                    servicer.RemoveVehicle,
                    request_deserializer=CarlaLink__pb2.Vehicle.FromString,
                    response_serializer=CarlaLink__pb2.Empty.SerializeToString,
            ),
            'UpdateVehicle': grpc.unary_unary_rpc_method_handler(
                    servicer.UpdateVehicle,
                    request_deserializer=CarlaLink__pb2.Vehicle.FromString,
                    response_serializer=CarlaLink__pb2.Empty.SerializeToString,
            ),
            'SimulationStep': grpc.unary_unary_rpc_method_handler(
                    servicer.SimulationStep,
                    request_deserializer=CarlaLink__pb2.Step.FromString,
                    response_serializer=CarlaLink__pb2.StepResult.SerializeToString,
            ),
            'GetTrafficLight': grpc.unary_unary_rpc_method_handler(
                    servicer.GetTrafficLight,
                    request_deserializer=CarlaLink__pb2.LandmarkRequest.FromString,
                    response_serializer=CarlaLink__pb2.TrafficLight.SerializeToString,
            ),
            'GetTrafficLightIDList': grpc.unary_unary_rpc_method_handler(
                    servicer.GetTrafficLightIDList,
                    request_deserializer=CarlaLink__pb2.Empty.FromString,
                    response_serializer=CarlaLink__pb2.TrafficLights.SerializeToString,
            ),
            'UpdateTrafficLight': grpc.unary_unary_rpc_method_handler(
                    servicer.UpdateTrafficLight,
                    request_deserializer=CarlaLink__pb2.TrafficLight.FromString,
                    response_serializer=CarlaLink__pb2.Empty.SerializeToString,
            ),
            'AddSensor': grpc.unary_unary_rpc_method_handler(
                    servicer.AddSensor,
                    request_deserializer=CarlaLink__pb2.Sensor.FromString,
                    response_serializer=CarlaLink__pb2.Sensor.SerializeToString,
            ),
            'RemoveSensor': grpc.unary_unary_rpc_method_handler(
                    servicer.RemoveSensor,
                    request_deserializer=CarlaLink__pb2.Sensor.FromString,
                    response_serializer=CarlaLink__pb2.Empty.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))


 # This class is part of an EXPERIMENTAL API.
class CarlaLinkService(object):
    """The greeting service definition.
    """

    @staticmethod
    def GetActor(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/GetActor',
            CarlaLink__pb2.ActorRequest.SerializeToString,
            CarlaLink__pb2.Vehicle.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def GetDepartedIDList(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/GetDepartedIDList',
            CarlaLink__pb2.Empty.SerializeToString,
            CarlaLink__pb2.DepartedActors.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def GetArrivedIDList(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/GetArrivedIDList',
            CarlaLink__pb2.Empty.SerializeToString,
            CarlaLink__pb2.ArrivedActors.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def AddVehicle(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/AddVehicle',
            CarlaLink__pb2.Vehicle.SerializeToString,
            CarlaLink__pb2.Empty.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def RemoveVehicle(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/RemoveVehicle',
            CarlaLink__pb2.Vehicle.SerializeToString,
            CarlaLink__pb2.Empty.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def UpdateVehicle(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/UpdateVehicle',
            CarlaLink__pb2.Vehicle.SerializeToString,
            CarlaLink__pb2.Empty.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def SimulationStep(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/SimulationStep',
            CarlaLink__pb2.Step.SerializeToString,
            CarlaLink__pb2.StepResult.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def GetTrafficLight(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/GetTrafficLight',
            CarlaLink__pb2.LandmarkRequest.SerializeToString,
            CarlaLink__pb2.TrafficLight.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def GetTrafficLightIDList(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/GetTrafficLightIDList',
            CarlaLink__pb2.Empty.SerializeToString,
            CarlaLink__pb2.TrafficLights.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def UpdateTrafficLight(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/UpdateTrafficLight',
            CarlaLink__pb2.TrafficLight.SerializeToString,
            CarlaLink__pb2.Empty.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def AddSensor(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/AddSensor',
            CarlaLink__pb2.Sensor.SerializeToString,
            CarlaLink__pb2.Sensor.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def RemoveSensor(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/org.eclipse.mosaic.fed.carla.grpc.CarlaLinkService/RemoveSensor',
            CarlaLink__pb2.Sensor.SerializeToString,
            CarlaLink__pb2.Empty.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)
