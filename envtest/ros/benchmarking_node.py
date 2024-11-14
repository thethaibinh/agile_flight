import argparse
import os
import yaml
import rospy
import numpy as np

from dodgeros_msgs.msg import QuadState
from envsim_msgs.msg import ObstacleArray
from std_msgs.msg import Empty
import open3d as o3d
from uniplot import plot

class Evaluator:
    def __init__(self, config, scenario, scene, policy, goal_x, goal_y, goal_z):
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
        if self.scene == 2:
            self.timer_check = rospy.Timer(rospy.Duration(1. / 20.), self.check_for_collision)

    def _initSubscribers(self, config):
        self.state_sub = rospy.Subscriber(
                "/%s/%s" % (config['quad_name'], config['state']),
                QuadState,
                self.callbackState,
                queue_size=1,
                tcp_nodelay=True)

        self.start_sub = rospy.Subscriber(
                "/%s/%s" % (config['quad_name'], config['start']),
                Empty,
                self.callbackStart,
                queue_size=1,
                tcp_nodelay=True)

        self.skip_trial_sub = rospy.Subscriber(
                "/%s/%s" % (config['quad_name'], config['skip_trial']),
                Empty,
                self.skip_trial,
                queue_size=1,
                tcp_nodelay=True)

    def _initPublishers(self, config):
        self.finish_pub = rospy.Publisher(
                "/%s/%s" % (config['quad_name'], config['finish']),
                Empty,
                queue_size=1,
                tcp_nodelay=True)

    def publishFinish(self):
        self.finish_pub.publish()
        self.printSummary()

    def callbackState(self, msg):
        if not self.is_active:
            return

        pos = np.array([msg.header.stamp.to_sec(),
                        msg.pose.position.x,
                        msg.pose.position.y,
                        msg.pose.position.z])
        self.pos.append(pos)
        self.current_pos = [msg.pose.position.x,
                            msg.pose.position.y,
                            msg.pose.position.z]

        pos_x = msg.pose.position.x
        bin_x = int(max(min(np.floor(pos_x),self.xmax),0))
        if np.isnan(self.time_array[bin_x]):
            self.time_array[bin_x] = rospy.get_rostime().to_sec()
        distance_to_goal = np.linalg.norm(pos[1:4] - ([self.xmax,self.ymax,self.zmax]))
        if distance_to_goal < 0.3:
            self.is_active = False
            self.publishFinish()

        if rospy.get_time() - self.time_array[0] > self.timeout:
            self.abortRun()

        outside = ((pos[1:] > self.bounding_box[1,:])
                    | (pos[1:] < self.bounding_box[0,:])).any(axis=-1)
        if (outside == True).any():
            self.abortRun()


    def callbackStart(self, msg):
        if self.scene == 2:
            cwd = os.getcwd()
            pointcloud_fname = os.path.join(cwd, "forest.ply")
            print("Reading pointcloud from %s" % pointcloud_fname)
            self.pcd = o3d.io.read_point_cloud(pointcloud_fname)
            if self.pcd is not None:
                print('Done reading the point cloud!')
            else:
                print('Failed to read the point cloud!')
            self.pcd_tree = o3d.geometry.KDTreeFlann(self.pcd)
            if self.pcd_tree is not None:
                print('Done converting into a KDTree!')
            else:
                print('Failed to convert into a KDTree!')
        else:
            rospy.sleep(1)
        if not self.is_active:
            self.is_active = True
        self.time_array[0] = rospy.get_rostime().to_sec()

    def check_for_collision(self, _timer):
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
            # it crashed into something, stop recording. Will not consider a condition to break the experiment now
            self.crash += 1
            print("Crashed")
            self.hit_obstacle = True
            # uncomment if you want to stop after crash
            self.abortRun()
        # make sure to not count double crashes
        if self.hit_obstacle and closest_distance > 2 * self.crashed_thr:
            self.hit_obstacle = False

    def abortRun(self):
        print("You did not reach the goal!")
        ttf = self.time_array[-1] - self.time_array[0]
        summary = {}
        summary['scenario'] = self.scenario
        summary['policy'] = self.policy
        summary['Success'] = False
        summary['time_to_finish'] = ttf
        pos = np.array(self.pos)
        _dist = 0
        for x in range(len(pos)):
            if x != 0:
                _dist += np.linalg.norm(pos[x,1:4] - pos[x-1,1:4])
        summary['travelled_distance'] = float(_dist)
        print("Your intermediate times are:")
        print_distance = 5
        summary['segment_times'] = {}
        for i in range(print_distance, self.xmax+1, print_distance):
            print("    %2i: %5.3fs " % (i,self.time_array[i] - self.time_array[0]))
            summary['segment_times']["%i" % i] = self.time_array[i] - self.time_array[0]
        print("You hit %i obstacles" % self.crash)
        summary['number_crashes'] = self.crash
        with open("../../evaluation.yaml", "r") as f:
            data = yaml.safe_load(f)
            rollout_name = 'rollout_1'
            if data is not None:
                items = list(data.items())
                if items[-1][0].split("_")[0] == 'rollout':
                    rollout_name = 'rollout_' + str(int(items[-1][0].split("_")[1]) + 1)
            f.close()

        with open("summary.yaml", "w") as f:
            tmp = {}
            tmp[rollout_name] = summary
            yaml.safe_dump(tmp, f)
        rospy.signal_shutdown("Completed Evaluation")

    def skip_trial(self, msg):
        self.is_skipped = True
        print("Skipping this trial due to simulation quality issues")
        with open("summary.yaml", "w") as f:
            f.write('')
        rospy.signal_shutdown("Skipped this trial")

    def printSummary(self):
        ttf = self.time_array[-1] - self.time_array[0]
        summary = {}
        summary['scenario'] = self.scenario
        summary['policy'] = self.policy
        summary['Success'] = True if self.crash == 0 else False
        print("You reached the goal in %5.3f seconds" % ttf)
        summary['time_to_finish'] = ttf
        pos = np.array(self.pos)
        _dist = 0
        for x in range(len(pos)):
            if x != 0:
                _dist += np.linalg.norm(pos[x,1:4] - pos[x-1,1:4])
        summary['travelled_distance'] = float(_dist)
        print("Your intermediate times are:")
        print_distance = 5
        summary['segment_times'] = {}
        for i in range(print_distance, self.xmax+1, print_distance):
            print("    %2i: %5.3fs " % (i,self.time_array[i] - self.time_array[0]))
            summary['segment_times']["%i" % i] = self.time_array[i] - self.time_array[0]
        print("You hit %i obstacles" % self.crash)
        summary['number_crashes'] = self.crash
        with open("../../evaluation.yaml", "r") as f:
            data = yaml.safe_load(f)
            rollout_name = 'rollout_1'
            if data is not None:
                items = list(data.items())
                if items[-1][0].split("_")[0] == 'rollout':
                    rollout_name = 'rollout_' + str(int(items[-1][0].split("_")[1]) + 1)
            f.close()

        with open("summary.yaml", "w") as f:
            tmp = {}
            tmp[rollout_name] = summary
            yaml.safe_dump(tmp, f)

        if not self.config['plots']:
            rospy.signal_shutdown("Completed Evaluation")
            return

        print("Here is a plot of your trajectory in the xy plane")
        # pos = np.array(self.pos)
        plot(xs=pos[:,1], ys=pos[:,2], color=True)

        print("Here is a plot of your average velocity per 1m x-segment")
        x = np.arange(1,self.xmax+1)
        dt = np.array(self.time_array)
        y = 1/(dt[1:]-dt[0:-1])
        plot(xs=x, ys=y, color=True)

        print("Here is a plot of the distance to the closest obstacles")
        dist = np.array(self.dist)
        plot(xs=dist[:,0]-self.time_array[0], ys=dist[:,1], color=True)

        rospy.signal_shutdown("Completed Evaluation")


if __name__=="__main__":
    rospy.init_node("evaluator", anonymous=False)  # Initialize the ROS node
    parser = argparse.ArgumentParser(description='Benchmarking node.')
    parser.add_argument('--policy', help='Navigation policy', required=False,  default='fixed_yawing')
    args = parser.parse_args()

    with open("./evaluation_config.yaml") as f:
        config = yaml.safe_load(f)

    with open("../../flightmare/flightpy/configs/vision/config.yaml") as f:
        scenario = yaml.safe_load(f)['environment']['level']

    with open("../../envtest/ros/planner/dna/planner_config.yaml") as f:
        planner_config = yaml.safe_load(f)
        goal_x = planner_config['goal_coordinate']['north']
        goal_y = planner_config['goal_coordinate']['west']
        goal_z = planner_config['goal_coordinate']['up']

    with open("../../flightmare/flightpy/configs/vision/config.yaml") as f:
        scene = yaml.safe_load(f)['unity']['scene_id']

    Evaluator(config, scenario, scene, args.policy, goal_x, goal_y, goal_z)  # Pass goal_x to the Evaluator
    rospy.spin()
