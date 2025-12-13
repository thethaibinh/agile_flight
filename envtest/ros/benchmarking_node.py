#!/usr/bin/env python3
import argparse
import os
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from dodgeros_msgs.msg import QuadState
from envsim_msgs.msg import ObstacleArray
from std_msgs.msg import Empty

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print("Warning: open3d not installed, collision checking disabled")

try:
    from uniplot import plot
    HAS_UNIPLOT = True
except ImportError:
    HAS_UNIPLOT = False
    print("Warning: uniplot not installed, plots disabled")


class Evaluator(Node):
    def __init__(self, config, scenario, scene, policy, goal_x, goal_y, goal_z):
        super().__init__('evaluator')
        
        self.policy = policy
        self.scenario = scenario
        self.scene = scene
        self.config = config
        self.pcd = None
        self.pcd_tree = None
        self.current_pos = None
        self.xmax = int(goal_x)
        self.ymax = int(goal_y)
        self.zmax = int(goal_z)
        self.is_active = False
        self.is_skipped = False
        self.pos = []
        self.dist = []
        self.time_array = (self.xmax+1)*[np.nan]

        self.hit_obstacle = False
        self.crash = 0
        self.timeout = self.config['timeout']
        self.bounding_box = np.reshape(np.array(
            self.config['bounding_box'], dtype=float), (3,2)).T
        self.bounding_box[1,0] = goal_x + 10
        self.crashed_thr = self.config['crashed_thr']

        self._initSubscribers(config['topics'])
        self._initPublishers(config['topics'])
        
        # Check at 20Hz the collision if we are in the forest
        if self.scene == 2 and HAS_OPEN3D:
            self.timer_check = self.create_timer(1.0 / 20.0, self.check_for_collision)

    def _initSubscribers(self, config):
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.state_sub = self.create_subscription(
            QuadState,
            "/%s/%s" % (config['quad_name'], config['state']),
            self.callbackState,
            qos)

        self.start_sub = self.create_subscription(
            Empty,
            "/%s/%s" % (config['quad_name'], config['start']),
            self.callbackStart,
            qos)

        self.skip_trial_sub = self.create_subscription(
            Empty,
            "/%s/%s" % (config['quad_name'], config['skip_trial']),
            self.skip_trial,
            qos)

    def _initPublishers(self, config):
        self.finish_pub = self.create_publisher(
            Empty,
            "/%s/%s" % (config['quad_name'], config['finish']),
            1)

    def publishFinish(self):
        self.finish_pub.publish(Empty())
        self.printSummary()

    def callbackState(self, msg):
        if not self.is_active:
            return

        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = np.array([stamp_sec,
                        msg.pose.position.x,
                        msg.pose.position.y,
                        msg.pose.position.z])
        self.pos.append(pos)
        self.current_pos = [msg.pose.position.x,
                            msg.pose.position.y,
                            msg.pose.position.z]

        pos_x = msg.pose.position.x
        bin_x = int(max(min(np.floor(pos_x), self.xmax), 0))
        
        current_time = self.get_clock().now().nanoseconds * 1e-9
        if np.isnan(self.time_array[bin_x]):
            self.time_array[bin_x] = current_time
            
        distance_to_goal = np.linalg.norm(pos[1:4] - ([self.xmax, self.ymax, self.zmax]))
        if distance_to_goal < 0.3:
            self.is_active = False
            self.publishFinish()

        if current_time - self.time_array[0] > self.timeout:
            self.abortRun()

        outside = ((pos[1:] > self.bounding_box[1,:])
                    | (pos[1:] < self.bounding_box[0,:])).any(axis=-1)
        if (outside == True).any():
            self.abortRun()

    def callbackStart(self, msg):
        if self.scene == 2 and HAS_OPEN3D:
            cwd = os.getcwd()
            pointcloud_fname = os.path.join(cwd, "forest.ply")
            self.get_logger().info("Reading pointcloud from %s" % pointcloud_fname)
            self.pcd = o3d.io.read_point_cloud(pointcloud_fname)
            if self.pcd is not None:
                self.get_logger().info('Done reading the point cloud!')
            else:
                self.get_logger().error('Failed to read the point cloud!')
            self.pcd_tree = o3d.geometry.KDTreeFlann(self.pcd)
            if self.pcd_tree is not None:
                self.get_logger().info('Done converting into a KDTree!')
            else:
                self.get_logger().error('Failed to convert into a KDTree!')
        else:
            import time
            time.sleep(1)
            
        if not self.is_active:
            self.is_active = True
        self.time_array[0] = self.get_clock().now().nanoseconds * 1e-9

    def check_for_collision(self):
        if not self.is_active:
            return
        if self.is_skipped:
            return

        # check if pointcloud is ready
        if self.pcd is None:
            return
        # Check if we have quadrotor state
        if self.current_pos is None:
            return

        # Number of crashes per maneuver
        [_, __, dist_squared] = self.pcd_tree.search_knn_vector_3d(self.current_pos, 1)
        closest_distance = np.sqrt(dist_squared)[0]

        if closest_distance < self.crashed_thr and (not self.hit_obstacle):
            # it crashed into something
            self.crash += 1
            self.get_logger().warn("Crashed")
            self.hit_obstacle = True
            self.abortRun()
        # make sure to not count double crashes
        if self.hit_obstacle and closest_distance > 2 * self.crashed_thr:
            self.hit_obstacle = False

    def abortRun(self):
        self.get_logger().info("You did not reach the goal!")
        ttf = self.time_array[-1] - self.time_array[0] if not np.isnan(self.time_array[-1]) else 0
        summary = {}
        summary['scenario'] = self.scenario
        summary['policy'] = self.policy
        summary['Success'] = False
        summary['time_to_finish'] = float(ttf)
        pos = np.array(self.pos) if self.pos else np.array([[0,0,0,0]])
        _dist = 0
        for x in range(len(pos)):
            if x != 0:
                _dist += np.linalg.norm(pos[x,1:4] - pos[x-1,1:4])
        summary['travelled_distance'] = float(_dist)
        self.get_logger().info("Your intermediate times are:")
        print_distance = 5
        summary['segment_times'] = {}
        for i in range(print_distance, self.xmax+1, print_distance):
            if not np.isnan(self.time_array[i]):
                self.get_logger().info("    %2i: %5.3fs " % (i, self.time_array[i] - self.time_array[0]))
                summary['segment_times']["%i" % i] = self.time_array[i] - self.time_array[0]
        self.get_logger().info("You hit %i obstacles" % self.crash)
        summary['number_crashes'] = self.crash
        
        eval_file = "../../../midi/evaluation.yaml"
        if os.path.exists(eval_file):
            with open(eval_file, "r") as f:
                data = yaml.safe_load(f)
                rollout_name = 'rollout_1'
                if data is not None:
                    items = list(data.items())
                    if items[-1][0].split("_")[0] == 'rollout':
                        rollout_name = 'rollout_' + str(int(items[-1][0].split("_")[1]) + 1)
        else:
            rollout_name = 'rollout_1'

        with open("summary.yaml", "w") as f:
            tmp = {}
            tmp[rollout_name] = summary
            yaml.safe_dump(tmp, f)
        
        raise SystemExit("Completed Evaluation")

    def skip_trial(self, msg):
        self.is_skipped = True
        self.get_logger().info("Skipping this trial due to simulation quality issues")
        with open("summary.yaml", "w") as f:
            f.write('')
        raise SystemExit("Skipped this trial")

    def printSummary(self):
        ttf = self.time_array[-1] - self.time_array[0] if not np.isnan(self.time_array[-1]) else 0
        summary = {}
        summary['scenario'] = self.scenario
        summary['policy'] = self.policy
        summary['Success'] = True if self.crash == 0 else False
        self.get_logger().info("You reached the goal in %5.3f seconds" % ttf)
        summary['time_to_finish'] = float(ttf)
        pos = np.array(self.pos) if self.pos else np.array([[0,0,0,0]])
        _dist = 0
        for x in range(len(pos)):
            if x != 0:
                _dist += np.linalg.norm(pos[x,1:4] - pos[x-1,1:4])
        summary['travelled_distance'] = float(_dist)
        self.get_logger().info("Your intermediate times are:")
        print_distance = 5
        summary['segment_times'] = {}
        for i in range(print_distance, self.xmax+1, print_distance):
            if not np.isnan(self.time_array[i]):
                self.get_logger().info("    %2i: %5.3fs " % (i, self.time_array[i] - self.time_array[0]))
                summary['segment_times']["%i" % i] = self.time_array[i] - self.time_array[0]
        self.get_logger().info("You hit %i obstacles" % self.crash)
        summary['number_crashes'] = self.crash
        
        eval_file = "../../../midi/evaluation.yaml"
        if os.path.exists(eval_file):
            with open(eval_file, "r") as f:
                data = yaml.safe_load(f)
                rollout_name = 'rollout_1'
                if data is not None:
                    items = list(data.items())
                    if items[-1][0].split("_")[0] == 'rollout':
                        rollout_name = 'rollout_' + str(int(items[-1][0].split("_")[1]) + 1)
        else:
            rollout_name = 'rollout_1'

        with open("summary.yaml", "w") as f:
            tmp = {}
            tmp[rollout_name] = summary
            yaml.safe_dump(tmp, f)

        if not self.config.get('plots', False) or not HAS_UNIPLOT:
            raise SystemExit("Completed Evaluation")

        print("Here is a plot of your trajectory in the xy plane")
        plot(xs=pos[:,1], ys=pos[:,2], color=True)

        print("Here is a plot of your average velocity per 1m x-segment")
        x = np.arange(1, self.xmax+1)
        dt = np.array(self.time_array)
        y = 1/(dt[1:]-dt[0:-1])
        plot(xs=x, ys=y, color=True)

        print("Here is a plot of the distance to the closest obstacles")
        dist = np.array(self.dist) if self.dist else np.array([[0, 0]])
        plot(xs=dist[:,0]-self.time_array[0], ys=dist[:,1], color=True)

        raise SystemExit("Completed Evaluation")


def main(args=None):
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser(description='Benchmarking node.')
    parser.add_argument('--policy', help='Navigation policy', required=False, default='fixed_yawing')
    parsed_args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    config_path = os.path.join(script_dir, "evaluation_config.yaml")
    with open(config_path) as f:
        config = yaml.safe_load(f)

    flightmare_config_path = os.path.join(script_dir, "../../../flightmare/flightpy/configs/vision/config.yaml")
    with open(flightmare_config_path) as f:
        scenario = yaml.safe_load(f)['environment']['level']

    planner_config_path = os.path.join(script_dir, "../../../midi/configs/sim.yaml")
    with open(planner_config_path) as f:
        planner_config = yaml.safe_load(f)
        goal_x = planner_config['goal_coordinate']['north']
        goal_y = planner_config['goal_coordinate']['west']
        goal_z = planner_config['goal_coordinate']['up']

    with open(flightmare_config_path) as f:
        scene = yaml.safe_load(f)['unity']['scene_id']

    node = Evaluator(config, scenario, scene, parsed_args.policy, goal_x, goal_y, goal_z)
    
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
