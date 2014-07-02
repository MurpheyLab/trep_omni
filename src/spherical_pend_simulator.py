#!/usr/bin/env python

"""
Jarvis Schultz

This node runs a timer that looks up the TF from the base link of the omni to
the end of the stylus. It then uses this pose to drive a trep simulation. The
trep simulation then provides force feedback. The position of the pendulum is
also published.

SUBSCRIBERS:
    - omni1_button (phantom_omni/PhantomButtonEvent)

PUBLISHERS:
    - mass_point (PointStamped)
    - visualization_marker_array (MarkerArray)

SERVICES:

"""

################
# ROS IMPORTS: #
################
import rospy
import tf
from tf import transformations as TR
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
import geometry_msgs.msg as GM
from phantom_omni.msg import PhantomButtonEvent
from std_msgs.msg import ColorRGBA
import visualization_msgs.msg as VM

###################
# NON-ROS IMPORTS #
###################
import trep
from trep import tx, ty, tz, rx, ry, rz
import numpy as np
import copy


####################
# GLOBAL CONSTANTS #
####################
DT = 1/100.
M = 0.25 # kg
L = 0.5 # m
B = 0.01 # damping
g = 9.81 #m/s^2
BASEFRAME = "base"
CONTFRAME = "stylus"
SIMFRAME = "trep_world"
MASSFRAME = "pend_mass"
NQ = 5
NU = 3

def build_system():
    system = trep.System()

    frames = [
        tx('xs', name='x-stylus', kinematic=True), [
            ty('ys', name='y-stylus', kinematic=True), [
                tz('zs', name='z-stylus', kinematic=True), [
                    rx('theta', name='roll'), [
                        ry('phi', name='pitch'), [
                            tz(-L, name=MASSFRAME, mass=M)]]]]]]
    system.import_frames(frames)
    trep.potentials.Gravity(system, (0,0,-g))
    trep.forces.Damping(system, B)
    return system


    
class PendSimulator:

    def __init__(self):
        rospy.loginfo("Creating PendSimulator class")
        # define initial config:
        self.q0 = np.zeros(NQ)
        # define running flag:
        self.running_flag = False
        self.grey_flag = False

        # setup markers
        self.setup_markers()
        
        # setup publishers, subscribers, timers:
        self.button_sub = rospy.Subscriber("omni1_button", PhantomButtonEvent,
                                           self.buttoncb)
        self.sim_timer = rospy.Timer(rospy.Duration(DT), self.timercb)
        self.mass_pub = rospy.Publisher("mass_point", PointStamped)
        self.marker_pub = rospy.Publisher("visualization_marker_array", VM.MarkerArray)
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        return

    def setup_markers(self):
        self.markers = VM.MarkerArray()
        # mass marker
        self.mass_marker = VM.Marker()
        self.mass_marker.action = VM.Marker.ADD
        self.mass_marker.color = ColorRGBA(*[1.0, 1.0, 1.0, 1.0])
        self.mass_marker.header.frame_id = SIMFRAME
        self.mass_marker.lifetime = rospy.Duration(5*DT)
        self.mass_marker.scale = GM.Vector3(*[0.05, 0.05, 0.05])
        self.mass_marker.type = VM.Marker.SPHERE
        self.mass_marker.id = 0
        # link marker
        self.link_marker = copy.deepcopy(self.mass_marker)
        self.link_marker.type = VM.Marker.LINE_STRIP
        self.link_marker.color = ColorRGBA(*[0.1, 0.1, 0.1, 1.0])
        self.link_marker.scale = GM.Vector3(*[0.005, 0.05, 0.05])
        self.link_marker.id = 1
        self.markers.markers.append(self.mass_marker)
        self.markers.markers.append(self.link_marker)
        return
    
        
    def setup_integrator(self):
        self.system = build_system()
        self.mvi = trep.MidpointVI(self.system)
        # self.mvi.initialize_from_configs(0, self.q0, DT, self.q0)
        self.mvi.initialize_from_state(0, self.q0, np.zeros(self.system.nQd))
        return


    def timercb(self, data):
        if not self.running_flag:
            return
        if self.listener.frameExists(SIMFRAME) and self.listener.frameExists(CONTFRAME):
            t = self.listener.getLatestCommonTime(SIMFRAME, CONTFRAME)
            try:
                position, quaternion = self.listener.lookupTransform(SIMFRAME, CONTFRAME, t)
            except (tf.Exception):
                rospy.logerr("Could not transform from "\
                             "{0:s} to {1:s}".format(SIMFRAME,CONTFRAME))
                return
        else:
            rospy.logerr("Could not find required frames "\
                         "for transformation from {0:s} to {1:s}".format(SIMFRAME,CONTFRAME))
            return
        # now we can use this position to integrate the trep simulation:
        ucont = np.zeros(NU)
        ucont[self.system.kin_configs.index(self.system.get_config('xs'))] = position[0]
        ucont[self.system.kin_configs.index(self.system.get_config('ys'))] = position[1]
        ucont[self.system.kin_configs.index(self.system.get_config('zs'))] = position[2]
        # step integrator:
        try:
            self.mvi.step(self.mvi.t2 + DT, k2=ucont)
        except trep.ConvergenceError as e:
            rospy.loginfo("Could not take step: %s"%e.message)
            return
        # if we successfully integrated, let's publish the point and the tf
        p = PointStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = SIMFRAME
        # get transform from trep world to mass frame:
        gwm = self.system.get_frame(MASSFRAME).g()
        ptrans = gwm[0:3, -1]
        # print ptrans
        p.point.x = ptrans[0]
        p.point.y = ptrans[1]
        p.point.z = ptrans[2]
        self.mass_pub.publish(p)
        # now we can send the transform:
        # ts = TransformStamped()
        # ts.header = p.header
        # ts.child_frame_id = MASSFRAME
        qtrans = TR.quaternion_from_matrix(gwm)
        # ts.transform.translation = geometry_msgs.msg.Vector3(*ptrans)
        # ts.transform.rotation = geometry_msgs.msg.Quaternion(*qtrans)
        self.br.sendTransform(ptrans, qtrans, p.header.stamp, MASSFRAME, SIMFRAME)

        # now we can publish the markers:
        for m in self.markers.markers:
            m.header.stamp = p.header.stamp
        self.mass_marker.pose = GM.Pose(position=GM.Point(*ptrans))
        p1 = GM.Point(*ptrans)
        p2 = GM.Point(*ucont)
        self.link_marker.points = [p1, p2]
        self.marker_pub.publish(self.markers)
        return
        
        

        
    def buttoncb(self, data):
        if data.grey_button == 1 and data.white_button == 0:
            rospy.loginfo("Integration primed")
            self.grey_flag = True
        elif data.grey_button == 0 and data.white_button == 0 and self.grey_flag == True:
            # then we previously pushed only the grey button, and we just released it
            rospy.loginfo("Starting integration")
            self.setup_integrator()
            self.running_flag = True
        else:
            rospy.loginfo("Integration stopped")
            self.grey_flag = False
            self.running_flag = False
        return



def main():
    """
    Run the main loop, by instatiating a System class, and then
    calling ros.spin
    """
    rospy.init_node('spherical_pend_sim', log_level=rospy.INFO)

    try:
        sim = PendSimulator()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
