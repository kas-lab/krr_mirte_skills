# Copyright 2025 KAS-Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import rclpy
import math
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from krr_mirte_skills_msgs.srv import PickObject, PlaceObject, GetObjectsInRoom
from gazebo_msgs.srv import GetEntityState, GetModelList
from boeing_gazebo_model_attachment_plugin_msgs.srv import Attach, Detach
from tf_transformations import euler_from_quaternion


class PickAndPlaceService(Node):
    def __init__(self):
        super().__init__("pick_service")

        self.cb_group_srv = MutuallyExclusiveCallbackGroup()
        self.cb_group_cli = MutuallyExclusiveCallbackGroup()

        self.pick_srv = self.create_service(
            PickObject,
            "pick_object",
            self.pick_callback,
            callback_group=self.cb_group_srv,
        )

        self.place_srv = self.create_service(
            PlaceObject,
            "place_object",
            self.place_callback,
            callback_group=self.cb_group_srv,
        )

        self.get_objects_cli = self.create_client(
            GetObjectsInRoom, "get_objects_in_room", callback_group=self.cb_group_cli
        )

        self.get_model_cli = self.create_client(
            GetModelList, "get_model_list", callback_group=self.cb_group_cli
        )

        self.get_entity_cli = self.create_client(
            GetEntityState, "get_entity_state", callback_group=self.cb_group_cli
        )

        self.attach_client = self.create_client(
            Attach, "/gazebo/attach", callback_group=self.cb_group_cli
        )

        self.detach_client = self.create_client(
            Detach, "/gazebo/detach", callback_group=self.cb_group_cli
        )

        self.robot_name = "mirte"
        self.pick_range = 0.5
        self.attached_object = None

    def pick_callback(self, request, response):
        object_id = getattr(request, "object_id", "").strip()

        if self.attached_object is not None:
            response.success = False
            response.error = f"Already holding {self.attached_object}"
            return response

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            response.success = False
            response.error = "Failed: Could not retrieve robot pose"
            return response

        model_list = self.get_model_list()
        if model_list is None:
            response.success = False
            response.error = "Failed: Could not retrieve model list"
            return response

        picked = False

        if object_id:
            matched_object = object_id
            obj_pose = self.get_model_pose(matched_object)
            if obj_pose is None:
                response.success = False
                response.error = f"Specified object {matched_object} not found"
                return response

            distance = self.calculate_distance(robot_pose, (obj_pose.x, obj_pose.y))
            if distance > self.pick_range or (
                self.is_object_behind_robot(robot_pose, (obj_pose.x, obj_pose.y)) and distance > 0.1
            ):
                response.success = False
                if self.is_object_behind_robot(robot_pose, (obj_pose.y, obj_pose.y)):
                    response.error = f"Specified object {matched_object} is behind the robot by distance ({distance:.2f}m)"
                response.error = (
                    f"Specified object {matched_object} is too far ({distance:.2f}m)"
                )
                return response

            if self.attach_object_to_gripper(matched_object):
                self.attached_object = matched_object
                response.success = True
                response.error = f"Picked object {matched_object}"
                picked = True
        else:
            object_poses = self.get_objects_in_room()
            if object_poses is None:
                response.success = False
                response.error = "Failed: Could not get objects from room"
                return response

            for obj_pose in object_poses:
                obj_x, obj_y = obj_pose.position.x, obj_pose.position.y
                distance = self.calculate_distance(robot_pose, (obj_x, obj_y))

                if distance > self.pick_range or (
                    self.is_object_behind_robot(robot_pose, (obj_x, obj_y))
                    and distance > 0.1
                ):
                    log_msg = (
                        f"Object at ({obj_x:.2f}, {obj_y:.2f}) is behind the robot by ({distance:.2f}m)!"
                        if self.is_object_behind_robot(robot_pose, (obj_x, obj_y))
                        else f"Object at ({obj_x:.2f}, {obj_y:.2f}) is too far ({distance:.2f}m)!"
                    )
                    self.get_logger().info(log_msg)
                    continue

                matched_object = self.match_pose_with_model(obj_pose, model_list)
                if matched_object and self.attach_object_to_gripper(matched_object):
                    self.attached_object = matched_object
                    response.success = True
                    response.error = f"Picked object {matched_object}"
                    picked = True
                    break

        if not picked:
            response.success = False
            response.error = "No suitable object within pick range"

        return response

    def get_robot_pose(self):
        request = GetEntityState.Request()
        request.name = self.robot_name
        result = self.call_service(self.get_entity_cli, request)

        if result and result.success:
            pos = result.state.pose.position
            orientation = result.state.pose.orientation
            _, _, yaw = euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w]
            )
            return (pos.x, pos.y, yaw)
        return None

    def is_object_behind_robot(self, robot_pose, obj_pose):
        robot_x, robot_y, robot_yaw = robot_pose
        obj_x, obj_y = obj_pose

        # Robot's forward direction vector based on yaw
        forward_x = math.cos(robot_yaw)
        forward_y = math.sin(robot_yaw)

        # Vector from robot to object
        vector_x = obj_x - robot_x
        vector_y = obj_y - robot_y

        # Dot product between robot's forward direction and object vector
        dot_product = forward_x * vector_x + forward_y * vector_y

        return dot_product < 0  # True if behind

    def get_objects_in_room(self):
        result = self.call_service(self.get_objects_cli, GetObjectsInRoom.Request())
        if result and result.success:
            objects_in_doorway_ = [doorway_object.objects_in_doorway for doorway_object in result.doorway_object_poses]
            objects_in_doorway = []
            for o in objects_in_doorway_:
                objects_in_doorway += o
            return result.room_object_poses + objects_in_doorway
        return None

    def get_model_list(self):
        result = self.call_service(self.get_model_cli, GetModelList.Request())
        return result.model_names if result else None

    def match_pose_with_model(self, obj_pose, model_list, threshold=0.1):
        closest_object = None
        min_distance = float("inf")

        for model_name in model_list:
            if model_name.startswith("obj_"):
                gazebo_pose = self.get_model_pose(model_name)
                if gazebo_pose is None:
                    continue
                distance = self.calculate_distance(
                    (obj_pose.position.x, obj_pose.position.y),
                    (gazebo_pose.x, gazebo_pose.y),
                )

                if distance < min_distance and distance < threshold:
                    min_distance = distance
                    closest_object = model_name

        return closest_object

    def get_model_pose(self, model_name):
        request = GetEntityState.Request()
        request.name = model_name
        result = self.call_service(self.get_entity_cli, request)

        if result and result.success:
            return result.state.pose.position
        return None

    @staticmethod
    def calculate_distance(pos1, pos2):
        return ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) ** 0.5

    def place_callback(self, request, response):
        if self.attached_object is None:
            response.success = False
            response.error = "No object to place"
            return response

        if self.detach_object_from_gripper(self.attached_object):
            response.success = True
            response.error = ""
            self.attached_object = None
        else:
            response.success = False
            response.error = "Failed: Could not detach object"

        return response

    def attach_object_to_gripper(self, object_id):
        request = Attach.Request(
            joint_name="test_joint",
            model_name_1=self.robot_name,
            link_name_1="Gripper",
            model_name_2=object_id,
            link_name_2="link",
        )

        return self.call_service(self.attach_client, request)

    def detach_object_from_gripper(self, object_id):
        request = Detach.Request()
        request.joint_name = "test_joint"
        request.model_name_1 = self.robot_name
        request.model_name_2 = object_id

        result = self.call_service(self.detach_client, request)
        return result is not None

    def call_service(self, client, request, timeout=5.0):
        future = client.call_async(request)
        self.executor.spin_until_future_complete(future, timeout_sec=timeout)
        return future.result() if future.done() else None


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceService()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
