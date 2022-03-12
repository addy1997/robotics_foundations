#!/usr/bin/env python
import sys
import copy
import rospy
import rospkg
import baxter_interface
import moveit_commander

from std_msgs.msg import (
    Empty,
)

from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

class PickAndPlaceMoveIt(object):
    def __init__(self, limb, hover_distance=0.15, verbose=True):
        self._limb_name = limb  # string
        self._hover_distance = hover_distance  # in meters
        self._verbose = verbose  # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")

        self._robot = moveit_commander.RobotCommander()
        # This is an interface to one group of joints.  In our case, we want to use the "right_arm".
        # We will use this to plan and execute motions
        self._group = moveit_commander.MoveGroupCommander(limb+"_arm")

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))

        self.gripper_open()
        self._group.set_pose_target(start_angles)
        plan = self._group.plan()
        self._group.execute(plan)
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance

        self._group.set_pose_target(approach)
        plan = self._group.plan()
        self._group.execute(plan)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w

        # servo up from current pose
        self._group.set_pose_target(ik_pose)
        plan = self._group.plan()
        self._group.execute(plan)

    def _servo_to_pose(self, pose):
        # servo down to release
        self._group.set_pose_target(pose)
        plan = self._group.plan()
        self._group.execute(plan)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

    def load_gazebo_models(table_pose=Pose(position=Point(x=1.0, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.68, y=0.11, z=0.7825)),
                       block_reference_frame="world"):
	    # Get Models' Path
	    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
	    # Load Table SDF
	    table_xml = ''
	    with open(model_path + "cafe_table/model.sdf", "r") as table_file:
	        table_xml = table_file.read().replace('\n', '')
	    # Spawn Table SDF
	    rospy.wait_for_service('/gazebo/spawn_sdf_model')
	    try:
	        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
	        spawn_sdf("cafe_table", table_xml, "/", table_pose, table_reference_frame)
	    except rospy.ServiceException, e:
	        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
	    # Spawn block SDF
	    rospy.wait_for_service('/gazebo/spawn_sdf_model')

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model("cafe_table")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ik_pick_and_place_moveit")

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'right'
    hover_distance = 0.15 # meters

   # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)

   # NOTE: Gazebo and Rviz has different origins, even though they are connected. For this
   # we need to compensate for this offset which is 0.93 from the ground in gazebo to
   # the actual 0, 0, 0 in Rviz.
    starting_pose = Pose(position=Point(x=0.75, y=0.235, z=0.35), orientation=overhead_orientation)
       
    pnp = PickAndPlaceMoveIt(limb, hover_distance)

    rospy.loginfo('The pick and place operation has started. The starting pose is %d',starting_pose)

    pnp.move_to_start(starting_pose)

    positions = rospy.get_param('piece_target_position_map')

    # reference to current positions from the piece_posemap
    current_piece_positions = ['00', '70', '20']
    rospy.loginfo('current positions of the blocks are %d', current_piece_positions)

    # reference to target positions from the piece_target_posemap
    target_piece_positions = ['04', '50', '21']
    rospy.loginfo('target positions of the blocks are %d', target_piece_positions)

    # list of poses to pick the blocks
    current_block_poses = list()

    # list of poses to place the blocks
    target_block_poses = list()

    # loop through each grid reference and add this to a list of block poses
    for pick in current_piece_positions:
        p = positions[pick]
        current_block_poses.append(Pose(position=Point(x=p[0], y=p[1], z=p[2]), orientation=overhead_orientation))
    
    for place in target_piece_positions:
        p = positions[place]
        target_block_poses.append(Pose(position=Point(x=p[0], y=p[1], z=p[2]), orientation=overhead_orientation))

    # loop over 5 iterations.
	for i in range(5):
	    if rospy.is_shutdown():
	        break

	    print("\nPicking...")
	    pnp.pick(current_block_poses[i])

	    print("\nPlacing...")
	    pnp.place(target_block_poses[i])
    return 0

if __name__ == '__main__':
    sys.exit(main())