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
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from krr_mirte_skills_msgs.srv import PickObject, PlaceObject, GetObjectsInRoom
from gazebo_msgs.srv import GetEntityState, GetModelList
from boeing_gazebo_model_attachment_plugin_msgs.srv import Attach, Detach


class PickAndPlaceService(Node):
    def __init__(self):
        super().__init__('pick_service')

        self.pick_srv = self.create_service(
            PickObject,
            'pick_object',
            self.pick_callback,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.place_srv = self.create_service(
            PlaceObject,
            'place_object',
            self.place_callback,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.get_objects_cli = self.create_client(
            GetObjectsInRoom,
            'get_objects_in_room',
            callback_group=MutuallyExclusiveCallbackGroup())

        self.get_model_cli = self.create_client(
            GetModelList,
            'get_model_list',
            callback_group=MutuallyExclusiveCallbackGroup())

        self.get_entity_cli = self.create_client(
            GetEntityState,
            'get_entity_state',
            callback_group=MutuallyExclusiveCallbackGroup())

        self.attach_client = self.create_client(
            Attach,
            '/gazebo/attach',
            callback_group=MutuallyExclusiveCallbackGroup())
        self.detach_client = self.create_client(
            Detach,
            '/gazebo/detach',
            callback_group=MutuallyExclusiveCallbackGroup())

        self.robot_name = "mirte"
        self.pick_range = 0.5
        self.attached_object = None

    def pick_callback(self, request, response):
        if self.attached_object is not None:
            self.get_logger().info(f"Already holding {self.attached_object}")
            response.success = False
            response.error = f"Already holding {self.attached_object}"
            return response

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            response.success = False
            response.error = "Failed: Could not retrieve robot pose"
            return response

        object_poses = self.get_objects_in_room()
        if object_poses is None:
            response.success = False
            response.error = "Failed: Could not get objects from room"
            return response

        model_list = self.get_model_list()
        if model_list is None:
            response.success = False
            response.error = "Failed: Could not retrieve model list"
            return response

        picked = False

        for obj_pose in object_poses:
            obj_x, obj_y = obj_pose.position.x, obj_pose.position.y
            distance = ((obj_x - robot_pose[0])**2 + (obj_y - robot_pose[1])**2)**0.5
            # if distance > self.pick_range or obj_x < -0.1:
            #     log_msg = f"Object {object_id} is behind the robot by ({distance:.2f}m)!" if obj_x < -0.1 else f"Object {object_id} is too far ({distance:.2f}m)!"
            #     self.get_logger().info(log_msg)
            #     response.success = False
            #     response.error = log_msg
            #     return response
            
            
            if distance <= self.pick_range and obj_x >= -0.1:
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
        if not self.get_entity_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("GetEntityState service unavailable!")
            return None

        request = GetEntityState.Request()
        request.name = self.robot_name

        future = self.get_entity_cli.call_async(request)
        self.executor.spin_until_future_complete(future, timeout_sec=5.0)

        if future.done() and future.result().success:
            pose = future.result().state.pose.position
            return (pose.x, pose.y)
        else:
            self.get_logger().error("Failed to retrieve robot pose")
            return None

    def get_objects_in_room(self):
        if not self.get_objects_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("GetObjectsInRoom service unavailable!")
            return None

        future = self.get_objects_cli.call_async(GetObjectsInRoom.Request())
        self.executor.spin_until_future_complete(future, timeout_sec=5.0)

        if future.done() and future.result().success:
            return future.result().object_poses
        else:
            self.get_logger().error("Failed to retrieve objects from room")
            return None

    def get_model_list(self):
        if not self.get_model_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("GetModelList service unavailable!")
            return None

        future = self.get_model_cli.call_async(GetModelList.Request())
        self.executor.spin_until_future_complete(future, timeout_sec=5.0)

        return future.result().model_names if future.done() else None

    def match_pose_with_model(self, obj_pose, model_list):
        for model_name in model_list:
            if model_name.startswith("obj_"):
                return model_name  # Simplified match
        return None

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
        request = Attach.Request()
        request.joint_name = "test_joint"
        request.model_name_1 = self.robot_name
        request.link_name_1 = "Gripper"
        request.model_name_2 = object_id
        request.link_name_2 = "link"

        result = self.call_service(self.attach_client, request)
        return result is not None

    def detach_object_from_gripper(self, object_id):
        request = Detach.Request()
        request.joint_name = "test_joint"
        request.model_name_1 = self.robot_name
        request.model_name_2 = object_id

        result = self.call_service(self.detach_client, request)
        return result is not None

    def call_service(self, cli, request):
        future = cli.call_async(request)
        self.executor.spin_until_future_complete(future, timeout_sec=5.0)
        return future.result() if future.done() else None


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
