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

from krr_mirte_skills_msgs.srv import GetObjectInfo
from gazebo_msgs.srv import GetModelProperties


class GetObjectInfoNode(Node):
    def __init__(self):
        super().__init__('get_object_info_service')
        self.srv = self.create_service(
            GetObjectInfo,
            'get_object_info',
            self.get_object_info_cb)
        self.cli_get = self.create_client(
            GetModelProperties,
            'get_model_properties',
            callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup())

    def get_object_info_cb(self, req, res):
        model_req = GetModelProperties.Request()
        model_req.model_name = "mirte"
        response = self.call_service(self.cli_get, model_req)

        object_name = next(
            (b for b in response.body_names if b.startswith("obj_")), None)
        if object_name is None or len(object_name.split('_')) < 3:
            res.success = False
            return res

        res.object_type = object_name.split('_')[2]
        res.success = True
        return res

    def call_service(self, cli, request):
        if cli.wait_for_service(timeout_sec=5.0) is False:
            self.get_logger().error(
                'service not available {}'.format(cli.srv_name))
            return None
        future = cli.call_async(request)
        self.executor.spin_until_future_complete(future, timeout_sec=5.0)
        if future.done() is False:
            self.get_logger().error(
                'Future not completed {}'.format(cli.srv_name))
            return None
        return future.result()


def main(args=None):
    rclpy.init(args=args)

    get_object_info_server = GetObjectInfoNode()
    rclpy.spin(get_object_info_server)


if __name__ == '__main__':
    main()
