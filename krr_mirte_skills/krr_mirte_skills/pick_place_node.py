import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from krr_mirte_skills_msgs.srv import PickObject, PlaceObject
from gazebo_msgs.srv import GetEntityState
from boeing_gazebo_model_attachment_plugin_msgs.srv import Attach, Detach

class PickAndPlaceService(Node):
    def __init__(self):
        super().__init__('pick_service')

        self.srv = self.create_service(PickObject, 'pick_object', self.pick_callback)
        self.place_srv = self.create_service(PlaceObject, 'place_object', self.place_callback)

        self.attached_object = None  

        # Gazebo services
        self.attach_client = self.create_client(Attach, '/gazebo/attach', callback_group=MutuallyExclusiveCallbackGroup())   
        self.detach_client = self.create_client(Detach, '/gazebo/detach', callback_group=MutuallyExclusiveCallbackGroup())   
        
        self.get_entity_cli = self.create_client(GetEntityState, 'get_entity_state', callback_group=MutuallyExclusiveCallbackGroup())
    


        self.robot_name = "mirte"  
        self.pick_range = 0.5  # Maximum allowed pick distance

    def pick_callback(self, request, response):
        object_id = request.object_id
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

    def place_callback(self, request, response):
        """ Handles the place service request """
        if self.attached_object is None:
            self.get_logger().info("No object is currently held, cannot place.")
            response.success = False
            response.error = "Failed: No object to place"
            return response

        # Get robot's pose to determine where to place the object
        # robot_pose = self.get_object_pose(self.robot_name)
        # if robot_pose is None:
        #     self.get_logger().error("Failed to get robot's position!")
        #     response.success = False
        #     response.error = "Failed: Could not retrieve robot pose"
        #     return response

        # Detach object in Gazebo to simulate placing it
        if self.detach_object_from_gripper(self.attached_object):
            self.get_logger().info(f"Successfully placed {self.attached_object}!")
            response.success = True
            response.error = ""
            self.attached_object = None  # clear the held obj
        else:
            response.success = False
            response.error = "Failed: Could not detach object"

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
    
    
    def attach_object_to_gripper(self, object_id):
        """ Calls Gazebo's attach service to attach the object to the gripper """
        if not self.attach_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Gazebo attach service unavailable!")
            return False

        request = Attach.Request()
        # todo: these needs to specify dynamically later
        request.joint_name = "test_joint"
        request.model_name_1 = self.robot_name
        request.link_name_1 = "Gripper"
        request.model_name_2 = object_id
        request.link_name_2 = "link"

        future = self.attach_client.call_async(request)
        while rclpy.spin_until_future_complete(self, future):
            self.get_logger().info("Waiting for future to complete")
        if future.result() is not None:
            self.get_logger().info(f"Attached {object_id} to gripper!")
            return True
        else:
            self.get_logger().error(f"Failed to attach {object_id}")
            return False
        
    def detach_object_from_gripper(self, object_id):
        """ Calls Gazebo's detach service to release the object from the gripper """
        if not self.detach_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Gazebo detach service unavailable!")
            return False
        request = Detach.Request()
        request.joint_name = "test_joint"
        request.model_name_1 = self.robot_name
        request.model_name_2 = object_id
        future = self.detach_client.call_async(request)
        while rclpy.spin_until_future_complete(self, future):
            self.get_logger().info("Waiting for future to complete")
        if future.result() is not None:
            self.get_logger().info(f"Detached {object_id} from gripper!")
            return True
        else:
            self.get_logger().error(f"Failed to detach {object_id}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceService()
    # rclpy.spin(node)
    # rclpy.shutdown()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as exception:
        traceback_logger.error(traceback.format_exc())
        raise exception
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
