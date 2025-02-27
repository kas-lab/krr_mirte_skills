import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from krr_mirte_skills_msgs.srv import PickObject, GetObjectsInRoom
from gazebo_msgs.srv import GetEntityState
from boeing_gazebo_model_attachment_plugin_msgs.srv import Attach

class PickService(Node):
    def __init__(self):
        super().__init__('pick_service')

        self.srv = self.create_service(PickObject, 'pick_object', self.pick_callback)

        self.attached_object = None  

        # Gazebo services
        self.attach_client = self.create_client(Attach, '/gazebo/attach', callback_group=MutuallyExclusiveCallbackGroup())   
        self.get_entity_cli = self.create_client(GetEntityState, 'get_entity_state', callback_group=MutuallyExclusiveCallbackGroup())
        self.get_entity_list_cli = self.create_client(GetObjectsInRoom, 'get_objects_in_room', callback_group=MutuallyExclusiveCallbackGroup())


        self.robot_name = "mirte"  
        self.pick_range = 0.5  # Maximum allowed pick distance

    def pick_callback(self, request, response):
        
        object_id = request.object_id
        
        # object_list = self.get_model_list()
        
        
        # Check if the gripper is already holding an object
        if self.attached_object is not None:
            self.get_logger().info(f"Gripper is already holding {self.attached_object}!")
            response.success = False
            response.error= f"Failed: Already holding {self.attached_object}"
            return response

        # Get the object's pose from Gazebo
        obj_pose = self.get_object_pose(object_id)
        if obj_pose is None:
            self.get_logger().info(f"Object {object_id} not found in Gazebo!")
            response.success = False
            response.error = "Failed: Object not found"
            return response

        # Get the robot's pose from Gazebo
        robot_pose = self.get_object_pose(self.robot_name)
        if robot_pose is None:
            self.get_logger().error("Failed to get robot's position!")
            response.success = False
            response.error = "Failed: Could not retrieve robot pose"
            return response

        # Compute distance between the robot and the object
        obj_x, obj_y, obj_z = obj_pose
        robot_x, robot_y, robot_z = robot_pose
        distance = ((obj_x - robot_x) ** 2 + (obj_y - robot_y) ** 2 + (obj_z - robot_z) ** 2) ** 0.5

        if distance > self.pick_range:
            self.get_logger().info(f"Object {object_id} is too far ({distance:.2f}m)!")
            response.success = False
            response.error = "Failed: Object out of range"
            return response

        # Attach object in Gazebo
        if self.attach_object_to_gripper(object_id):
            self.attached_object = object_id
            response.success = True
            response.error = f"Success: Picked {object_id}"
        else:
            response.success = False
            response.error = "Failed: Could not attach object"

        return response

    def get_object_pose(self, entity_name):
        """ Calls Gazebo's get_entity_state service to get an entity's position """
        if not self.get_entity_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Gazebo get_entity_state service unavailable!")
            return None

        request = GetEntityState.Request()
        request.name = entity_name

        future = self.get_entity_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is None or not result.success:
            self.get_logger().error(f"Failed to get state of {entity_name}")
            return None

        return (result.state.pose.position.x, result.state.pose.position.y, result.state.pose.position.z)

    def get_model_list(self):
        """ Calls Gazebo's service to get the list of all models in the world """
        if not self.get_entity_list_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Gazebo get_objects_in_room service unavailable!")
            return None

        request = GetObjectsInRoom.Request()

        future = self.get_entity_list_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is None or not result.entity_names:
            self.get_logger().error("Failed to retrieve model list from Gazebo!")
            return None

        self.get_logger().info(f"Objects in Gazebo: {result.entity_names}")
        return result.entity_names
    
    
    def attach_object_to_gripper(self, object_id):
        """ Calls Gazebo's attach service to attach the object to the gripper """
        if not self.attach_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Gazebo attach service unavailable!")
            return False

        request = Attach.Request()
        request.joint_name = "test_joint"
        request.model_name_1 = self.robot_name
        request.link_name_1 = "Gripper"
        request.model_name_2 = object_id
        request.link_name_2 = "link"

        future = self.attach_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Attached {object_id} to gripper!")
            return True
        else:
            self.get_logger().error(f"Failed to attach {object_id}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = PickService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
