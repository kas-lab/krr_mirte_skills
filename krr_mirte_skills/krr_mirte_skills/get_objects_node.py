import sys

from gazebo_msgs.srv import GetModelList, GetEntityState
from krr_mirte_skills_msgs.msg import DropLocation
from krr_mirte_skills_msgs.srv import GetDropLocations, GetObjectsInRoom
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import numpy as np


def is_entity_in_room(entity_state, corners):

    ex, ey = entity_state.pose.position.x, entity_state.pose.position.y

    inside = True
    for i in range(4):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) % 4]

        edge_vector = (x2 - x1, y2 - y1)

        point_vector = (ex - x1, ey - y1)

        cross = edge_vector[0] * point_vector[1] - \
            edge_vector[1] * point_vector[0]

        # Determine if the point is consistently on the same side of all edges
        if i == 0:
            reference_sign = np.sign(cross)
        else:
            if np.sign(cross) != reference_sign and cross != 0:
                inside = False
                break

    return inside


class GetObjectsInRoomNode(Node):

    DROP_ENTITY_PREFIX = "drop_"
    GRABABLE_ENTITY_PREFIX = "obj_"
    ROBOT_ENTITY_NAME = "mirte"
    rooms = {
        "kitchen": [(-3.06, 2.04), (2.71, 1.95), (2.77, -1.94), (-3.07, -1.92)],
        "living_room": [(-3.15, -4.84), (-9.56, -4.79), (-9.36, 1.88), (-3.25, 1.81)],
        "office": [(-3.25, 2.04), (-9.36, 2.11), (-9.35, 7.37), (-3.24, 7.37)],
        "bedroom": [(2.76, 2.18), (-3.02, 2.05), (-2.99, 7.35), (2.76, 7.38)]
    }

    def __init__(self):
        super().__init__('get_objects_in_room_node')
        self.srv = self.create_service(
            GetObjectsInRoom,
            'get_objects_in_room',
            self.get_objects_callback,
            callback_group=MutuallyExclusiveCallbackGroup())
        self.srv_drop = self.create_service(
            GetDropLocations,
            'get_drop_locations',
            self.get_drop_locations_callback,
            callback_group=MutuallyExclusiveCallbackGroup())
        self.get_model_cli = self.create_client(
            GetModelList,
            'get_model_list',
            callback_group=MutuallyExclusiveCallbackGroup())
        self.get_entity_cli = self.create_client(
            GetEntityState,
            'get_entity_state',
            callback_group=MutuallyExclusiveCallbackGroup())
        self.ent_req = GetEntityState.Request()
        self.get_logger().info("Node created")

    def which_room(self, entity_state):
        room_checklist = [
            (room[0],
             is_entity_in_room(
                entity_state,
                room[1])) for room in list(
                self.rooms.items())]

        if sum([is_in_room for _, is_in_room in room_checklist]) != 1:
            return None  # In no or multiple rooms.

        return next(
            (name for name, is_in_room in room_checklist if is_in_room))

    def send_request_async(self, client, request):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = client.call_async(request)

        while rclpy.spin_until_future_complete(self, future):
            self.get_logger().info("Waiting for future to complete")

        return future.result()

    def get_objects_callback(self, _, response):
        response.success = False
        self.get_logger().info("Get models")

        entity_names = self.send_request_async(
            self.get_model_cli, GetModelList.Request()).model_names

        self.get_logger().info("Got model list")
        self.get_logger().info(str(entity_names))

        grabable_entities = [
            entity_name for entity_name in entity_names if entity_name.startswith(
                self.GRABABLE_ENTITY_PREFIX)]

        self.ent_req.name = self.ROBOT_ENTITY_NAME
        self.get_logger().info("Get entity state")
        self.get_logger().info(str(self.ent_req))

        robot_entity_state = self.send_request_async(
            self.get_entity_cli, self.ent_req)
        self.get_logger().info("Got robot entity")
        self.get_logger().info(str(robot_entity_state.success))

        if(robot_entity_state.success):
            current_room = self.which_room(robot_entity_state.state)

            if(current_room is None):
                return response

            entities_in_room = []
            for entity_name in grabable_entities:
                self.ent_req.name = entity_name

                obj_ent_state = self.send_request_async(
                    self.get_entity_cli, self.ent_req)

                if(not obj_ent_state.success):
                    return response

                if(is_entity_in_room(obj_ent_state.state, self.rooms[current_room])):
                    entities_in_room.append(obj_ent_state.state.pose)

            response.success = True
            response.object_poses = entities_in_room

        return response

    def get_drop_locations_callback(self, _, response):

        entity_names = self.send_request_async(
            self.get_model_cli, GetModelList.Request()).model_names
        drop_entities = [
            entity_name for entity_name in entity_names if entity_name.startswith(
                self.DROP_ENTITY_PREFIX)]

        for entity_name in drop_entities:
            self.ent_req.name = entity_name
            obj_ent_state = self.send_request_async(
                self.get_entity_cli, self.ent_req)

            drop_location = DropLocation()
            drop_location.drop_pose = obj_ent_state.state.pose
            drop_location.type.data = entity_name.split('_')[2]
            response.drop_locations.append(drop_location)

        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)

    node = GetObjectsInRoomNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
