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

class GetObjectInfoNode(Node):
  def __init__(self):
    super().__init__('get_object_info_service')
    self.srv = self.create_service(
      GetObjectInfo,
      'get_object_info',
      self.get_object_info_cb)

  def get_object_info_cb(self, req, res):
    # TODO: something to check which object the robot is holding
    
    name = "obj_3_spoon"
    res.object_type = name.split('_')[2]
    res.success = True
    return res
  
def main(args=None):
  rclpy.init(args=args)

  get_object_info_server = GetObjectInfoNode()

  rclpy.spin(get_object_info_server)


if __name__ == '__main__':
  main()