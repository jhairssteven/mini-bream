import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Trigger

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from tf_transformations import quaternion_from_euler

class InteractivePathNode(Node):
    def __init__(self):
        super().__init__('interactive_path_node')

        self.server = InteractiveMarkerServer(self, "interactive_path_markers")
        self.path_pub = self.create_publisher(Path, "/rviz_path", 10)

        # Service to trigger path publishing
        self.srv = self.create_service(Trigger, 'rviz_path', self.handle_rviz_path)

        default_positions = [
            [[-50.247276306152344, -139.0640411376953, 1.9073486328125e-06], [0.0, 0.0, -0.9998044967651367, 0.01977263018488884]],
            [[-65.58441925048828, -146.8583984375, 0.0], [0.0, 0.0, -0.8125176429748535, 0.5829366445541382]],
            [[-67.28180694580078, -152.09947204589844, 0.0], [0.0, 0.0, -0.8039597868919373, 0.5946836471557617]],
        ]
        self.num_markers = len(default_positions)
        for i, pos in enumerate(default_positions):
            self.create_interactive_marker(i, pos[0], pos[1])

        # Optional: Create a button marker to trigger the service
        self.create_publish_button()

    def create_interactive_marker(self, index, pos=[0, 0, 0], quat=[0,1,0,1]):
        x, y, _ = pos
        q_x, q_y, q_z, q_w = quat
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "world"
        int_marker.name = f"pose_{index}"
        int_marker.description = f"Pose {index}"
        int_marker.scale = 1.0
        int_marker.pose.position.x = x
        int_marker.pose.position.y = y
        int_marker.pose.position.z = 0.0
        int_marker.pose.orientation.x = q_x
        int_marker.pose.orientation.y = q_y
        int_marker.pose.orientation.z = q_z
        int_marker.pose.orientation.w = q_w
        

        # 🔵 Create a visible sphere for dragging
        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = 0.3
        sphere_marker.scale.y = 0.3
        sphere_marker.scale.z = 0.3
        sphere_marker.color.r = 0.1
        sphere_marker.color.g = 0.4
        sphere_marker.color.b = 0.9
        sphere_marker.color.a = 1.0

        # 🔁 Control to drag in x–y plane
        move_control = InteractiveMarkerControl()
        move_control.name = "move_xy"
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        move_control.always_visible = True
        move_control.markers.append(sphere_marker)

        # 💡 Set control orientation so Z points up (to make plane = XY)
        move_control.orientation.w = 1.0
        move_control.orientation.x = 0.0
        move_control.orientation.y = 1.0
        move_control.orientation.z = 0.0

        int_marker.controls.append(move_control)

          # 🔄 Control for rotating around Z (yaw)
        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "rotate_z"
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        rotate_control.orientation.w = 1.0
        rotate_control.orientation.x = 0.0
        rotate_control.orientation.y = 1.0
        rotate_control.orientation.z = 0.0
        int_marker.controls.append(rotate_control)

        self.server.insert(int_marker)
        self.server.setCallback(int_marker.name, self.marker_feedback)
        self.server.applyChanges()

    def create_publish_button(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "world"
        int_marker.name = "publish_button"
        int_marker.description = "Click to Publish Path"
        int_marker.scale = 1.0
        int_marker.pose.position.x = 0.0
        int_marker.pose.position.y = -2.0
        int_marker.pose.position.z = 0.0

        # 🔵 Create a visible cube marker
        cube_marker = Marker()
        cube_marker.type = Marker.CUBE
        cube_marker.scale.x = 0.3
        cube_marker.scale.y = 0.3
        cube_marker.scale.z = 0.3
        cube_marker.color.r = 0.2
        cube_marker.color.g = 0.8
        cube_marker.color.b = 0.2
        cube_marker.color.a = 1.0

        # 🔘 Create the button control with the cube
        button_control = InteractiveMarkerControl()
        button_control.name = "button"
        button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        button_control.always_visible = True
        button_control.markers.append(cube_marker)

        int_marker.controls.append(button_control)

        self.server.insert(int_marker)
        self.server.setCallback(int_marker.name, self.on_button_click)
        self.server.applyChanges()

    def marker_feedback(self, feedback):
        self.get_logger().info(f"Marker {feedback.marker_name} moved.")

    def on_button_click(self, feedback):
        if feedback.event_type == feedback.BUTTON_CLICK:
            self.get_logger().info("Button clicked — calling publish_path()")
            self.publish_path()

    def handle_rviz_path(self, request, response):
        self.publish_path()
        response.success = True
        response.message = "Path published."
        return response

    def publish_path(self):
        path = Path()
        path.header.frame_id = "world"
        path.header.stamp = self.get_clock().now().to_msg()

        for i in range(self.num_markers):
            marker = self.server.get(f"pose_{i}")
            if marker:
                pose_stamped = PoseStamped()
                pose_stamped.header = path.header
                pose_stamped.pose = marker.pose
                path.poses.append(pose_stamped)

        self.path_pub.publish(path)
        self.get_logger().info(f"Published path with {len(path.poses)} poses.")

def main(args=None):
    rclpy.init(args=args)
    node = InteractivePathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
