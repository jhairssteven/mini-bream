#!/usr/bin/env python3

import sys
import rospy
from tf.transformations import quaternion_from_euler
from geographic_msgs.msg import GeoPath, GeoPoseStamped
from CommonUtils import *
from NavigationTools import *
from Robot import Robot
from path_planner.msg import DoMissionGoal

LOOP_FREQ = 5
Animal2Action = {"crocodile": "avoid", "platypus": "cw", "turtle": "ccw"}
Animal2RadiusIdeal = {"crocodile": 13, "platypus": 4, "turtle": 4}
Animal2RadiusCritical = {"crocodile": 11, "platypus": 2, "turtle": 2}
Animal2Status = {"crocodile": True, "platypus": False, "turtle": False}


class Animal:
    def __init__(self, name, pose, index):
        self.name = name
        self.pose = pose
        self.index = index
        self.action = Animal2Action[self.name]
        self.radius_ideal = Animal2RadiusIdeal[self.name]
        self.radius_critical = Animal2RadiusCritical[self.name]
        self.radius_actual = self.radius_ideal
        self.boat_approaching = True
        self.boat_circumventing = False
        self.finished = Animal2Status[self.name]
        # self.midway_stop_time = 0


class AngleAccumulator:
    def __init__(self, yaw=0):
        self.cur_yaw = yaw
        self.accumulated_yaw = 0

    def update(self, yaw):
        self.accumulated_yaw += norm(yaw - self.cur_yaw)
        self.cur_yaw = yaw

    def reset(self, yaw):
        self.cur_yaw = yaw
        self.accumulated_yaw = 0


class Task4:
    def __init__(self):
        self.robot = Robot(enable_gps=True, enable_imu=True, enable_task_info=True, enable_client=True)

        # Record all animals as a list
        self.animals = []
        self.animals_count = 0

        # Circumvention around degree check class
        self.angle_accumulator = AngleAccumulator()

        # Receive animals gps info
        self.wildlife_gps_sub = rospy.Subscriber('/vrx/wildlife/animals/poses', GeoPath, self.wildlife_gps_callback,
                                                 queue_size=1)

        # Reserved for direct control of boat by local polar info
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def wildlife_gps_callback(self, data):
        for idx, animal in enumerate(data.poses):
            name = animal.header.frame_id
            pose = NavigationTools.Pose(gps_lat=animal.pose.position.latitude, gps_lon=animal.pose.position.longitude)
            # Use Pose2D for CommonUtils compatibility
            pose2d = Pose2D()
            pose2d.x, pose2d.y = pose.utm_x, pose.utm_y  # heading of animals is not concerned here

            # update pose only after all animals are determined in this run
            if self.animals_count:
                self.animals[idx].pose = pose2d
            else:
                self.animals.append(Animal(name, pose, idx))
        self.animals_count = len(self.animals)

    def next_waypoint(self):
        """
        Core function that contains the waypoint-choosing logic to finish the task
        :return:
        """
        rho_new, alpha_new, beta_new = 0, 0, 0

        # Make sure animals' poses are received to proceed
        if self.animals_count == 0:
            rospy.loginfo("Waiting for animals' poses to update next waypoint...")
            return rho_new, alpha_new, beta_new
        rospy.loginfo_once("Got %d animals: ", self.animals_count)
        rospy.loginfo_once("\t%s", self.animals[0].name)
        rospy.loginfo_once("\t%s", self.animals[1].name)
        rospy.loginfo_once("\t%s", self.animals[2].name)

        # Make sure boat's pose is received to proceed
        if self.robot.boat_cartpose == Pose2D():
            rospy.loginfo("Waiting for boat cartesian pose to update next waypoint...")
            return rho_new, alpha_new, beta_new

        # Choose animal to circumvent
        next_index = self._get_next_animal_index()
        if next_index is None:
            rospy.logwarn("No animal to circumvent!")
            all_crocodiles = all([animal.action == "avoid" for animal in self.animals])
            assert not all_crocodiles
            return rho_new, alpha_new, beta_new
        assert self.animals[next_index].action != "avoid"
        rospy.loginfo("Trying to circumvent %s %s.", self.animals[next_index].name,
                               self.animals[next_index].action)

        # Update safe circumvent radius of target animal
        self._update_safe_radius(next_index)

        # Get distance and heading angle towards target animal position center
        rho, alpha, _ = get_rho_alpha_beta_cart(self.robot.boat_cartpose, self.animals[next_index].pose)

        # Get params for boat to head towards tangent line of circle of that animal
        rho_new, alpha_new, beta_new = self._choose_waypoint(rho, alpha,
                                                             self.animals[next_index].radius_actual,
                                                             clockwise=self.animals[next_index].action == "cw",
                                                             animal_index=next_index)

        # Add potential deviation to heading angle to avoid crocodile(s) in the route
        # Rho is unchanged here because as long as it is nonzero, it will keep the boat moving forward
        # Beta seems to have little effect here so also remains unchanged
        if not self.animals[next_index].boat_circumventing and not self.near_to_animal(next_index, max_dist=25.0):
            rospy.loginfo("Original alpha: %f, original rho: %f", alpha_new, rho_new)
            alpha_new = self._get_safe_alpha(alpha_new, clockwise=self.animals[next_index].action == "cw")
            # limit rho to decrease boat linear velocity when new alpha is large
            # to make boat stay outside crocodile circle
            rho_new = min(3 / abs(alpha_new), rho_new)
            rospy.loginfo("Updated alpha: %f, updated rho: %f", alpha_new, rho_new)

        # mark animal properties when finished circumventing
        if self._check_around(self.angle_accumulator, next_index):
            rospy.loginfo("Has circumvented %s for over a round.", self.animals[next_index].name)
            self.animals[next_index].finished = True
            self.animals[next_index].boat_approaching = False
            self.animals[next_index].boat_circumventing = False
            self.angle_accumulator.reset(0)

            # unload boat inertia to better approach next animal
            self.keep_pose(duration=3.)
        elif self.animals[next_index].boat_circumventing:
            rospy.loginfo("Has circumvented %s for %f degrees.",
                          self.animals[next_index].name, rad2deg(self.angle_accumulator.accumulated_yaw))

        rospy.loginfo("Next local polar waypoint (%fm, %fdeg, %fdeg)", rho_new, rad2deg(alpha_new), rad2deg(beta_new))
        return rho_new, alpha_new, beta_new

    def near_to_animal(self, animal_index, max_dist=20.0):
        animal_pose = self.animals[animal_index].pose
        dist = get_dist(self.robot.boat_cartpose, animal_pose)
        return dist < max_dist

    def _get_next_animal_index(self):
        """
        Choose the nearest circumvent animal to navigate to
        :return: target animal index
        """
        index = None  # by default an invalid index
        min_dist = 1000000
        it = 0
        while it < self.animals_count:
            # Crocodile should not be circumvented
            if self.animals[it].action == "avoid":
                it += 1
                continue
            # Avoid switching animal when circumventing
            if self.animals[it].boat_circumventing:
                return it
            # Animal should not be circumvented twice
            if self.animals[it].finished:
                it += 1
                continue
            dist = get_dist(self.robot.boat_cartpose, self.animals[it].pose)
            if dist < min_dist:
                min_dist = dist
                index = it
            it += 1
        return index

    def _update_safe_radius(self, animal_index):
        assert animal_index < len(self.animals)

        for idx, animal in enumerate(self.animals):
            if idx == animal_index:
                continue
            rho, alpha, _ = get_rho_alpha_beta_cart(self.robot.boat_cartpose, self.animals[animal_index].pose)
            rho_avoid, alpha_avoid, _ = get_rho_alpha_beta_cart(self.robot.boat_cartpose, self.animals[idx].pose)
            delta_alpha = abs(alpha - alpha_avoid)
            circum2avoid_dist = m.sqrt(rho ** 2 + rho_avoid ** 2 - 2 * rho * rho_avoid * m.cos(delta_alpha))
            if animal.action == "avoid":
                # Try to stay outside ideal radius of crocodile and critical radius of platypus/turtle
                r = circum2avoid_dist - self.animals[idx].radius_ideal
                if r > self.animals[animal_index].radius_ideal:
                    r = self.animals[animal_index].radius_ideal
                if r < self.animals[animal_index].radius_critical:
                    r = self.animals[animal_index].radius_critical
                self.animals[animal_index].radius_actual = r
            else:
                # Action is either "cw" or "ccw"
                # Try to stay outside critical radius of the other platypus/turtle while circumventing
                # consider radius of all circumventing animals separately to accommodate to different moving
                # speeds in the future
                r = circum2avoid_dist - self.animals[animal_index].radius_ideal
                if r > self.animals[idx].radius_critical:
                    self.animals[idx].radius_actual = self.animals[idx].radius_critical
                    self.animals[animal_index].radius_actual = self.animals[animal_index].radius_ideal
                elif circum2avoid_dist - self.animals[animal_index].radius_critical < \
                        self.animals[idx].radius_critical:
                    rospy.logwarn("Two circumventing animals staying too close!")
                    self.animals[animal_index].radius_actual = self.animals[animal_index].radius_critical
                else:
                    self.animals[animal_index].radius_actual = self.animals[animal_index].radius_critical

        rospy.loginfo("Actual radius of %s is %f m", self.animals[animal_index].name,
                               self.animals[animal_index].radius_actual)

    def _get_safe_alpha(self, orig_alpha, clockwise):
        """
        Choose alpha to avoid crocodile circular zone
        :param orig_alpha: original waypoint's alpha in rad
               clockwise: target animal circumvent direction
        :return: adjusted safe alpha in rad
        """
        safe_alpha = orig_alpha
        avoid_indexes = [animal.index for animal in self.animals if animal.action == "avoid"]
        if len(avoid_indexes) == 0:  # no need to adjust alpha if no crocodiles
            return safe_alpha
        assert len(avoid_indexes) < 3

        merge_theta_range = False
        if len(avoid_indexes) == 2 and \
                get_dist(self.animals[avoid_indexes[0]].pose, self.animals[avoid_indexes[1]].pose) < \
                self.animals[avoid_indexes[0]].radius_ideal + self.animals[avoid_indexes[1]].radius_ideal:
            merge_theta_range = True

        alpha_ranges = {}  # index to angle ranges
        nearest_rho_index = 0
        nearest_rho = 1000000
        for idx in avoid_indexes:
            rho, alpha, _ = get_rho_alpha_beta_cart(self.robot.boat_cartpose, self.animals[idx].pose)
            if rho < nearest_rho:
                nearest_rho = rho
                nearest_rho_index = idx
            r = self.animals[idx].radius_ideal
            _, alpha1, _ = self._choose_waypoint(rho, alpha, r, clockwise=False, animal_index=idx)
            _, alpha2, _ = self._choose_waypoint(rho, alpha, r, clockwise=True, animal_index=idx)

            if alpha1 == alpha2:
                rospy.loginfo("use last alpha!")
                safe_alpha = alpha1
                return safe_alpha

            # align 2 alphas ccw
            if norm(alpha2 - alpha1) > 0:
                alpha_ranges[idx] = (alpha1, alpha2)
            else:
                alpha_ranges[idx] = (alpha2, alpha1)

        if len(alpha_ranges) == 0:
            rospy.loginfo("No obstacles in front view!")
            return safe_alpha
        print("Alpha ranges for all crocodiles: ")
        print(alpha_ranges)

        # Two crocodiles' circles overlap, boat cannot traverse between them
        if merge_theta_range:
            alpha_ranges = [(min(alpha_ranges[avoid_indexes[0]][0], alpha_ranges[avoid_indexes[1]][0]),
                             max(alpha_ranges[avoid_indexes[0]][1], alpha_ranges[avoid_indexes[1]][1]))]

        # If alpha does not cross any circle, it will stay unchanged
        # If alpha crosses 1 circle, choose the nearest edge alpha of that circle
        # If alpha crosses 2 circles, choose the nearest edge alpha of the nearest (rho) circle
        # In both cases, choose the nearest alpha of an alpha range
        within_count = 0
        for key, value in alpha_ranges.items():
            if within(safe_alpha, value[0], value[1]):
                rospy.loginfo("Current alpha %f is within %f and %f", safe_alpha, value[0], value[1])
                within_count += 1
                if within_count == 2:
                    # safe_alpha = nearest_edge(safe_alpha, alpha_ranges[nearest_rho_index][0],
                    #                           alpha_ranges[nearest_rho_index][1])

                    safe_alpha = alpha_ranges[nearest_rho_index][1] if clockwise else alpha_ranges[nearest_rho_index][0]
                    break
                else:
                    # safe_alpha = nearest_edge(safe_alpha, value[0], value[1])

                    # fix chosen safe alpha by target animal's circumvention direction
                    # choose left alpha if want to circumvent cw, otherwise right alpha
                    safe_alpha = value[1] if clockwise else value[0]

                # adjust alpha further to repel boat more away from crocodile
                safe_alpha += deg2rad(10) if clockwise else -deg2rad(10)
                safe_alpha = norm(safe_alpha)
        return safe_alpha

    def _choose_waypoint(self, rho, alpha, radius, clockwise=True, animal_index=None):
        """
        Head towards the tangent point of a circle if far from it, circumvent a circle if near it
        :param rho: distance to circle center in meter
        :param alpha: heading error wrt center center in rad
        :param radius: radius of circle in meter
        :param clockwise: circumvent direction
        :return: Tuple (rho, alpha, beta) of newly selected tangent point
        """
        if animal_index is None:
            rospy.logwarn("Need to specify animal when choosing waypoint!")
            return 0, 0, 0
        r_inner = radius
        r_outer = 11.
        rho_new = rho
        alpha_new = alpha
        beta_new = 0
        if rho > 10 and self.animals[animal_index].boat_approaching:
            if self.animals[animal_index].action != "avoid":
                rospy.loginfo("Approaching %s ... rho: %f, rho_inner: %f, rho_outer: %f",
                              self.animals[animal_index].name, rho, r_inner, r_outer)

                # if rho < 20 and self.animals[animal_index].midway_stop_time == 0:
                #     self.keep_pose(duration=3)
                #     self.animals[animal_index].midway_stop_time += 1

                rho_new = m.sqrt(rho ** 2 - r_inner ** 2)  # avoid arcsin error
                r_inner = r_inner if r_inner < rho else rho
                if clockwise:
                    alpha_new = alpha + m.asin(r_inner / rho)
                    # beta_new = -deg2rad(10)
                else:
                    alpha_new = alpha - m.asin(r_inner / rho)
                    # beta_new = deg2rad(10)
            else:
                rospy.loginfo("Avoiding %s ... rho: %f, rho_inner: %f, rho_outer: %f", self.animals[animal_index].name, rho, r_inner, r_outer)
                if r_inner < rho:
                    if clockwise:
                        alpha_new = alpha + m.asin(r_inner / rho)
                        # beta_new = -deg2rad(10)
                    else:
                        alpha_new = alpha - m.asin(r_inner / rho)
                        # beta_new = deg2rad(10)
                else:
                    rospy.loginfo("Inside circle, move along the previous tangent line.")
                    r_inner = rho - 1
                    if clockwise:
                        alpha_new = alpha + m.asin(r_inner / rho)
                        # beta_new = -deg2rad(10)
                    else:
                        alpha_new = alpha - m.asin(r_inner / rho)
                        # beta_new = deg2rad(10)
        else:
            rospy.loginfo("Circumventing %s ... rho to animal center: %f", self.animals[animal_index].name, rho)
            if self.animals[animal_index].boat_approaching:
                # unload boat inertia everytime when boat starts to circumvent
                self.keep_pose(duration=2.0)

                rospy.loginfo("Start to accumulate angle!")
                self.angle_accumulator.reset(self.robot.boat_cartpose.theta)
                self.animals[animal_index].boat_approaching = False

            if rho > r_outer:
                self.animals[animal_index].boat_circumventing = False
                self.animals[animal_index].boat_approaching = True
                rospy.loginfo("Out of range when circumventing!")
            else:  # accumulate angle when circumventing within some bigger radius
                self.animals[animal_index].boat_circumventing = True
                self.angle_accumulator.update(self.robot.boat_cartpose.theta)
            rho_new = m.sqrt(rho ** 2 + radius ** 2)
            if clockwise:
                beta_new = -m.atan2(radius, rho)
            else:
                beta_new = m.atan2(radius, rho)
            alpha_new = alpha - beta_new
        alpha_new = norm(alpha_new)
        return rho_new, alpha_new, beta_new

    def _check_around(self, accumulator, animal_index):
        """
        Check if the boat has circumvented the animal for a round
        :param accumulator: un-normalized yaw angle
        :param animal_index: currently circumventing animal index
        :return:
        """
        if self.animals[animal_index].action == "cw":
            # rospy.loginfo("cw accumulated yaw is %f", accumulator.accumulated_yaw)
            return accumulator.accumulated_yaw < -2.0 * m.pi
        elif self.animals[animal_index].action == "ccw":
            # rospy.loginfo("ccw accumulated yaw is %f", accumulator.accumulated_yaw)
            return accumulator.accumulated_yaw > 2.0 * m.pi
        else:  # crocodile needs not to be checked
            rospy.logwarn("Check around for wrong animal %s", self.animals[animal_index])
            return False

    def _to_ros_geopose(self, pose):
        """
        Convert custom-defined geo-pose to ros GeoPoseStamped
        :param geopose: NavigationTools.Pose
        :return: geographic_msgs.GeoPoseStamped
        """
        geopose = GeoPoseStamped()
        geopose.header.stamp = rospy.Time.now()
        geopose.header.frame_id = "REGULAR"  # hard code the waypoint as regular
        geopose.pose.position.latitude = pose.gps_lat
        geopose.pose.position.longitude = pose.gps_lon
        q = quaternion_from_euler(0, 0, pose.head)
        geopose.pose.orientation.x = q[0]
        geopose.pose.orientation.y = q[1]
        geopose.pose.orientation.z = q[2]
        geopose.pose.orientation.w = q[3]
        return geopose

    def task_completed(self):
        """
        All platypus and turtles been circumvented is deemed as task completed
        :return:
        """
        if self.animals_count == 0:
            return False
        states = [animal.finished for animal in self.animals]
        return all(states)

    def pub_twist(self, rho, alpha, beta):
        """
        Get and publish twist by local polar waypoint
        :param rho:
        :param alpha:
        :param beta:
        :return:
        """
        # for T config
        # t = get_twist(rho, alpha, beta, kr=0.08, ka=1.0, kb=0.1, max_linear_x=0.8, max_angular_z=1.4)

        # for X config
        t = get_twist(rho, alpha, beta, kr=0.07, ka=0.8, kb=0.1, max_linear_x=1, max_angular_z=0.8)
        self.twist_pub.publish(t)

    def keep_pose(self, duration=4.0):
        boat_geopose_stamped = self.robot.to_ros_geopose(self.robot.boat_geopose, wp_type="POSE_KEEP")
        goals = DoMissionGoal(mission=[boat_geopose_stamped, boat_geopose_stamped])
        self.robot.action_client.send_goal(goals)
        self.robot.action_client.wait_for_result(timeout=rospy.Duration.from_sec(duration))
        self.robot.action_client.cancel_all_goals()
        rospy.loginfo("Pose keep finished!")

    def run(self):
        """
        Main loop for this wildlife task
        :return:
        """
        while not self.robot.is_data_ready():
            rospy.loginfo("Waiting for sensor data to be ready...")
            rospy.sleep(rospy.Duration(secs=1))
        rospy.loginfo("All sensor data is ready!")

        r = rospy.Rate(LOOP_FREQ)
        while not rospy.is_shutdown():
            r.sleep()

            if self.robot.task_info.state == "ready":
                rospy.loginfo_once("Task is ready.")
                # TODO could do something if needed
                continue
            if self.robot.task_info.state == "running":
                rospy.loginfo_once("Start running!")
                # Get next local polar waypoint
                rho, alpha, beta = self.next_waypoint()

                # Convert local polar waypoint to cartesian frame (utm actually) waypoint
                x, y, theta = get_goal_cart_pose(self.robot.boat_cartpose, rho, alpha, beta)
                rospy.loginfo("Boat utm   x: %fm y: %fm theta: %fdeg",
                              self.robot.boat_cartpose.x, self.robot.boat_cartpose.y,
                              rad2deg(self.robot.boat_cartpose.theta))
                rospy.loginfo("Goal utm   x: %fm y: %fm theta: %fdeg", x, y, rad2deg(theta))

                # Convert utm frame waypoint to spherical frame geo pose (lla with heading)
                # Assume boat and goal poses are in the same utm zone
                goal_pose = NavigationTools.Pose(utm_x=x, utm_y=y, utm_zone=self.robot.boat_geopose.utm_zone, head=theta)
                goal_geopose_stamped = self._to_ros_geopose(goal_pose)
                boat_geopose_stamped = self._to_ros_geopose(self.robot.boat_geopose)

                # Include boat current pose as the first waypoint
                # since path planner requires at least 2 geo poses to plan a path
                # goals = DoMissionGoal(mission=[boat_geopose_stamped, goal_geopose_stamped])
                # self.robot.action_client.send_goal(goals)

                # Reserved direct control of boat
                # Cancel action request and uncomment pub_twist will enable local polar PID control
                self.pub_twist(rho, alpha, beta)
            if self.task_completed():
                rospy.loginfo("Task completed!")
                break


if __name__ == "__main__":
    # Required ROS initialization
    rospy.init_node('wildlife', anonymous=True)
    rospy.set_param("dubins_radius", 5.0)
    rospy.set_param("dubins_step_size", 0.5)
    task_4 = Task4()
    try:
        task_4.run()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)
