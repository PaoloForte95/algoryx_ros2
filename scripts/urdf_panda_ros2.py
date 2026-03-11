"""
Example scene with a model of the Franka Emika Panda robot loaded as a URDF model and controlled via ROS2

Controlling the panda robot:
    Writing float32 values to topics 'jointX_speed' (replace X with the joint number) will make the
    joints rotate with the specified speed and writing float32 values to topic 'gripper_pos' will set
    the position of the gripper.
"""

import agx
import agxSDK
import agxOSG
import agxCollide
import agxRender
import agxModel
import agxPython
import agxROS2
from agxROS2 import ROS2ControlInterface as cpp_ROS2ControlInterface, ROS2ClockPublisher
import agxUtil
from rclpy.node import Node

import sys
import os
import numpy as np
from PIL import Image
from athena_msgs.msg import PlanningProblem
from rclpy.utilities import get_default_context

from agxPythonModules.utils.environment import (
    is_running_tests,
)

try:
    os.environ["QT_LOGGING_RULES"] = "qt.qpa.*=false;qt.core.qobject.connect=false"  # Silence annoying warnings
    from PySide6 import QtWidgets, QtGui

    if is_running_tests():
        QtWidgets = None
        QtGui = None
except:  # noqa
    print("Could not find PySide6. Continuing without displaying images")
    QtWidgets = None
    QtGui = None


SAVE_IMAGE = False

from agxPythonModules.utils.environment import simulation, root, application, init_app
from agxPythonModules.sensors.camera_sensors import (
    VirtualCameraSensor,
    VirtualCameraSensorRBFollower,
)



class DisplayVirtualCameraGUIListener(agxSDK.StepEventListener):
    """
    PySide6 gui for displaying a list of virtual camera sensors
    """

    def __init__(self, virtual_cameras):
        super().__init__(agxSDK.StepEventListener.LAST_STEP)
        self._virtual_cameras = virtual_cameras

        self._qt_app = QtWidgets.QApplication([])
        self._qt_window = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()
        self._qt_button = QtWidgets.QPushButton("Quit!")
        self._qt_button.clicked.connect(sys.exit)
        layout.addWidget(self._qt_button)
        layout2 = QtWidgets.QHBoxLayout()
        layout.addLayout(layout2)
        self.latest_png = None
        self.image_pub = agxROS2.PublisherSensorMsgsImage("/camera/camera/color/image_raw")


        self._qt_labels = []
        for _ in virtual_cameras:
            label = QtWidgets.QLabel("No camera feed")
            layout2.addWidget(label)
            self._qt_labels.append(label)

        self._qt_window.setLayout(layout)
        self._qt_window.show()

    def last(self, t):
        for c, l in zip(self._virtual_cameras, self._qt_labels):
            q_image = self.get_qt_image(c.image)
            l.setPixmap(QtGui.QPixmap(q_image))
        global SAVE_IMAGE
        if(SAVE_IMAGE):
            Image.fromarray(self.latest_png, mode="RGB").save("rgb2.png")
            SAVE_IMAGE = False
        self._qt_app.processEvents()
        self.publish_image()
    
        

    def publish_image(self):
        if self.latest_png is None:
            return

        img = np.require(self.latest_png, np.uint8, "C")
        h, w, c = img.shape

        image_msg = agxROS2.SensorMsgsImage()
        image_msg.height = h
        image_msg.width = w
        image_msg.encoding = "rgb8"
        image_msg.is_bigendian = 0
        image_msg.step = c * w
        image_msg.data = img.tobytes()
        self.image_pub.sendMessage(image_msg)

    def get_qt_image(self, img: np.ndarray):
        # Ensure numpy array
        img = np.asarray(img)

        # Convert floats [0,1] -> uint8 [0,255]
        if img.dtype != np.uint8:
            # If values are already 0..255 floats, clip still works fine
            img = np.clip(img * 255.0, 0, 255).astype(np.uint8)

        # Handle channel shapes safely
        if img.ndim == 2:
            # (H, W) -> (H, W, 3)
            img = np.repeat(img[:, :, None], 3, axis=2)

        elif img.ndim == 3:
            # (H, W, C)
            if img.shape[2] == 1:
                # (H, W, 1) -> (H, W, 3)
                img = np.repeat(img, 3, axis=2)
            elif img.shape[2] >= 3:
                img = img[:, :, :3]
            else:
                raise ValueError(f"Unsupported channel count: {img.shape[2]}")

        elif img.ndim == 4:
            # Common accidental shape (H, W, 1, 3) or (1, H, W, 3)
            if img.shape[-2:] == (1, 3):          # (H, W, 1, 3)
                img = img[:, :, 0, :]
            elif img.shape[0] == 1 and img.shape[-1] == 3:  # (1, H, W, 3)
                img = img[0]
            else:
                raise ValueError(f"Unsupported 4D image shape: {img.shape}")

        else:
            raise ValueError(f"Unsupported image ndim: {img.ndim}, shape: {img.shape}")

        # Ensure contiguous memory
        img = np.require(img, np.uint8, "C")
        h, w, _ = img.shape
        bytes_per_line = 3 * w

        if img.ndim == 3 and img.shape[2] == 3 and img.dtype == np.uint8:
            self.latest_png = img.copy()  

        # IMPORTANT: keep a reference so QImage doesn't point to freed memory
        qimg = QtGui.QImage(img.data, w, h, bytes_per_line, QtGui.QImage.Format.Format_RGB888)
        qimg.ndarray = img  # attach to keep alive

        return qimg
        
class TimerListener(agxSDK.StepEventListener):
    def __init__(self, decorator):
        super().__init__(agxSDK.StepEventListener.PRE_STEP)
        self.decorator = decorator

    def pre(self, t):
        self.decorator.setText(
            0, "Time: {:3.2f}".format(t), agx.Vec4f(0.7, 0.7, 0.7, 1)
        )



# Class representing the Panda robot
class PandaRobot(agxSDK.StepEventListener):
    def __init__(self, panda_assembly):
        super().__init__()

        self.ros_sub_joint_speeds = agxROS2.SubscriberStdMsgsFloat32MultiArray("joint_speeds")
        self.ros_sub_gripper_pos = agxROS2.SubscriberStdMsgsFloat32("gripper_pos")
        self.ros_sub_reset_pos = agxROS2.SubscriberStdMsgsFloat32("reset_pos")
        self.ros_pub_finger_pos = agxROS2.PublisherStdMsgsFloat32("finger_pos")
        self.ros_sub_planning_problem = agxROS2.SubscriberStdMsgsString("planning_problem")

        self.msg_received_float32_multiarray = agxROS2.StdMsgsFloat32MultiArray()
        self.msg_received_float32 = agxROS2.StdMsgsFloat32()
        self.msg_received_string = agxROS2.StdMsgsString()

 
        
        self.panda = panda_assembly

        self._joints = []
        self._body_transformations = []

        self.init()

    def reset(self):
        print("Reset")
        for b, t in self._body_transformations:
            b.setTransform(t)
            b.setVelocity(agx.Vec3(0, 0, 0))
            b.setAngularVelocity(agx.Vec3(0, 0, 0))

        for c in self._joints:
            c.rebind()

    def init(self): 
        for c in self.panda.getConstraints():
            self._joints.append(c)
            

        for b in self.panda.getRigidBodies():
            self._body_transformations.append([b, b.getTransform()])

        # Enable Motor1D (speed controller) on all joints, except the gripper for which we
        # enable the Lock1D (pos controller) instead
        def enable_motor1d(self, joint_name):
            self.panda.getConstraint1DOF(joint_name).getMotor1D().setEnable(True)
            self.panda.getConstraint1DOF(joint_name).getMotor1D().setForceRange(-1000, 1000)

        enable_motor1d(self, "fr3_joint1")
        enable_motor1d(self, "fr3_joint2")
        enable_motor1d(self, "fr3_joint3")
        enable_motor1d(self, "fr3_joint4")
        enable_motor1d(self, "fr3_joint5")
        enable_motor1d(self, "fr3_joint6")
        enable_motor1d(self, "fr3_joint7")
        self.panda.getConstraint1DOF("fr3_finger_joint1").getLock1D().setEnable(True)
        self.panda.getConstraint1DOF("fr3_finger_joint2").getLock1D().setEnable(True)






    def pre(self, time):
        if self.ros_sub_joint_speeds.receiveMessage(self.msg_received_float32_multiarray):
            self.received_joint_speeds(self.msg_received_float32_multiarray)
        if self.ros_sub_gripper_pos.receiveMessage(self.msg_received_float32):
            self.received_gripper_pos(self.msg_received_float32)
        if self.ros_sub_reset_pos.receiveMessage(self.msg_received_float32):
            self.received_reset_pos(self.msg_received_float32)
        if self.ros_sub_planning_problem.receiveMessage(self.msg_received_string):
            global SAVE_IMAGE
            SAVE_IMAGE = True
        

    # def post(self, time):
    #     self.publish_finger_pos()

        
    def publish_finger_pos(self):
        gripper_pos = self.panda.getConstraint1DOF("panda_finger_joint1").getAngle()
        msg = agxROS2.StdMsgsFloat32()
        msg.data = gripper_pos
        self.ros_pub_finger_pos.sendMessage(msg)
    
   
    # Received new ros message, update speed of joint 1
    def received_joint_speeds(self, msg):
        self.panda.getConstraint1DOF("fr3_joint1").getMotor1D().setSpeed(msg.data[0])
        self.panda.getConstraint1DOF("fr3_joint2").getMotor1D().setSpeed(msg.data[1])
        self.panda.getConstraint1DOF("fr3_joint3").getMotor1D().setSpeed(msg.data[2])
        self.panda.getConstraint1DOF("fr3_joint4").getMotor1D().setSpeed(msg.data[3])
        self.panda.getConstraint1DOF("fr3_joint5").getMotor1D().setSpeed(msg.data[4])
        self.panda.getConstraint1DOF("fr3_joint6").getMotor1D().setSpeed(msg.data[5])
        self.panda.getConstraint1DOF("fr3_joint7").getMotor1D().setSpeed(msg.data[6])

    # Received new ros message, update pos of the gripper
    def received_gripper_pos(self, msg):
        self.panda.getConstraint1DOF("fr3_finger_joint1").getLock1D().setPosition(msg.data)
        self.panda.getConstraint1DOF("fr3_finger_joint2").getLock1D().setPosition(msg.data)

    # Received new ros message, reset all joints
    def received_reset_pos(self, msg):
        self.reset()


def setupCamera(app):
    cameraData = app.getCameraData()
    cameraData.eye = agx.Vec3(3.0813, 1.4021, 1.4828)
    cameraData.center = agx.Vec3(-0.0716, -0.0077, 0.4658)
    cameraData.up = agx.Vec3(-0.2857, -0.0514, 0.9569)
    cameraData.nearClippingPlane = 0.1
    cameraData.farClippingPlane = 5000
    app.applyCameraData(cameraData)


def buildScene():
    app = agxPython.getContext().environment.getApplication()
    sim = agxPython.getContext().environment.getSimulation()
    root = agxPython.getContext().environment.getSceneRoot()


    table_size = agx.Vec3(0.3, 0.3, 0.3)

    table = agxCollide.Geometry(agxCollide.Box(table_size))
    table.setPosition(agx.Vec3(0.5, 0, 0.3))
    sim.add(table)
    sim.add(TimerListener(app.getSceneDecorator()))
    agxOSG.createVisual(table, root)
    agxOSG.setTexture(table, root, "textures/grid.png")


    # Side view
    #dx = 0.5    
    #eye    = agx.Vec3(dx, -0.3, 1.2)
    #center = agx.Vec3(-0.1,  -0.3, 0.0)
    #up     = agx.Vec3(0.0,  0.0, 1.0)
    
    # Top view
    dx = 0.5    
    dy = 0.4   # positive = forward, negative = backward
    eye    = agx.Vec3(0.0 + dx, -0.3 + dy, 1.2)
    center = agx.Vec3(-0.1 + dx, -0.3 + dy, 0.0)
    up     = agx.Vec3(0.0,  0.0, 1.0)

     # Ceates a virtual camera sensor with a specified eye, center and up direction.
    virtual_camera_rgbd = VirtualCameraSensor(
        640,
        480,
        eye,
        center,
        up,
        fovy=64,
        near=0.1,
        far=10.0,
        depth_camera=True,
    )
    # A step event listener that attaches the camera to a body and makes it follow the body
    follower1 = VirtualCameraSensorRBFollower(
        virtual_camera_rgbd,
        eye,
        center,
        up,
        table
    )
    sim.add(follower1)


    virtual_camera_rgb = VirtualCameraSensor(
        640,
        480,
        eye,
        center,
        up,
        fovy=64,
        near=0.1,
        far=10.0,
        depth_camera=False,
    )

    # A step event listener that attaches the camera to a body and makes it follow the body
    follower2 = VirtualCameraSensorRBFollower(
        virtual_camera_rgbd,
        eye,
        center,
        up,
        table
    )
    sim.add(follower2)

    if QtWidgets is not None:
        display_virtual_camera_image = DisplayVirtualCameraGUIListener(
        [virtual_camera_rgbd,virtual_camera_rgb]
    )
    sim.add(display_virtual_camera_image)

    trimesh_apple = agxUtil.createTrimesh(
            "src/algoryx/meshes/apple/apple.dae",
            agxCollide.Trimesh.REMOVE_DUPLICATE_VERTICES,
            agx.Matrix3x3(agx.Vec3(0.025, 0.025, 0.025))
            * agx.Matrix3x3(agx.EulerAngles(agx.PI_2, 0, 0)),
        )
    rb_apple = agx.RigidBody()
    rb_apple.setPosition(table.getPosition() + agx.Vec3(0.15, -0.15, 0.3))
    geo_apple = agxCollide.Geometry(trimesh_apple)
    rb_apple.add(geo_apple)



    trimesh_banana = agxUtil.createTrimesh("src/algoryx/meshes/banana/banana.obj", 
                                           agxCollide.Trimesh.REMOVE_DUPLICATE_VERTICES,
                                           agx.Matrix3x3(agx.Vec3(0.015, 0.015, 0.015))
                                            )
    
    rb_banana = agx.RigidBody()
    rb_banana.setPosition(table.getPosition() + agx.Vec3(0.0, 0.15, 0.3))
    geo_banana = agxCollide.Geometry(trimesh_banana)
    rb_banana.add(geo_banana)
    
    sim.add(rb_apple)
    sim.add(rb_banana)
    apple_visual = agxOSG.createVisual(geo_apple, root)
    banana_visual = agxOSG.createVisual(geo_banana, root)
    agxOSG.setDiffuseColor(apple_visual, agxRender.Color.Red())
    agxOSG.setDiffuseColor(banana_visual, agxRender.Color.Yellow())

    # Construct the floor that the panda robot will stand on
    floor = agxCollide.Geometry(agxCollide.Box(agx.Vec3(1, 1, 0.1)))
    floor.setPosition(0, 0, -0.1)
    sim.add(floor)
    fl = agxOSG.createVisual(floor, root)
    agxOSG.setDiffuseColor(fl, agxRender.Color.LightGray())

    # Set the sky color
    application().getSceneDecorator().setBackgroundColor(
        agxRender.Color.SkyBlue(), agxRender.Color.DodgerBlue()
    )

    # Read the URDF file
    urdf_file = os.path.join("src/algoryx_ros2/urdf/fr3.urdf")
    package_path = os.path.join("/home/pofe/franka_ros2_ws/src/")

    # Determines if the base link of the URDF model should be attached to the
    # world or not. Default is false.
    fixToWorld = True

    # If true the collision between linked/constrained bodies will be disabled.
    # Default is false.
    disableLinkedBodies = True

    # If true, any link missing the <inertial></inertial> key will be treated
    # as a kinematic link and will be merged with its parent. According to http://wiki.ros.org/urdf/XML/link
    # If false, the body will get its mass properties calculated from the shape/volume/density.
    # Default is True.
    mergeKinematicLink = False

    settings = agxModel.UrdfReaderSettings(fixToWorld, disableLinkedBodies, mergeKinematicLink)

    init_joint_angles = agx.RealVector()
    init_joint_angles.append(round(0.0, 3))
    init_joint_angles.append(round(0.0, 3))
    init_joint_angles.append(round(0.0, 3))
    init_joint_angles.append(round(-0.08, 3))
    init_joint_angles.append(round(0.05, 3))
    init_joint_angles.append(round(1.57, 3))
    init_joint_angles.append(round(0.785, 3))
    gripper_start_pos = round(0.03, 2)
    init_joint_angles.append(gripper_start_pos)
    init_joint_angles.append(gripper_start_pos)

    panda_assembly_ref = agxModel.UrdfReader.read(urdf_file, package_path, init_joint_angles, settings)

    if panda_assembly_ref.get() is None:
        print("Error reading the URDF file.")
        sys.exit(2)

    # Disable self-collision between links.
    sim.getSpace().setEnablePair(panda_assembly_ref.getName(), panda_assembly_ref.getName(), False)

    # Add the panda assembly to the simulation and create visualization for it
    sim.add(panda_assembly_ref.get())
    fl = agxOSG.createVisual(panda_assembly_ref.get(), root)

    # Create the Panda robot representation with a ROS2 subscriber
    panda = PandaRobot(panda_assembly_ref.get())

    #panda.panda.getConstraint1DOF("panda_finger_joint1").getLock1D().setPosition(0.01)
    #panda.panda.getConstraint1DOF("panda_finger_joint2").getLock1D().setPosition(0.01)

    sim.add(panda)

    ros2_clock = ROS2ClockPublisher(9)
    sim.add(ros2_clock, agxSDK.EventManager.HIGHEST_PRIORITY)

    

    panda_arm_control_interface = cpp_ROS2ControlInterface(
        "agx_joint_commands",
        "agx_joint_states",
        syncWithSystemTime=True,
        domainId=9
    )

    control_joint_names = [
        "fr3_joint1",
        "fr3_joint2",
        "fr3_joint3",
        "fr3_joint4",
        "fr3_joint5",
        "fr3_joint6",
        "fr3_joint7",
    ]

    for name in control_joint_names:
        if not panda_arm_control_interface.addJoint(panda.panda.getConstraint1DOF(name), agxROS2.ROS2ControlInterface.VELOCITY):
            print("Could not add joint ", name)

    panda_arm_control_interface.addJoint(panda.panda.getConstraint1DOF("fr3_finger_joint1"), agxROS2.ROS2ControlInterface.POSITION)
    panda_arm_control_interface.addJoint(panda.panda.getConstraint1DOF("fr3_finger_joint2"), agxROS2.ROS2ControlInterface.POSITION)
    sim.add(panda_arm_control_interface)

    # Setup the camera
    setupCamera(app)

    application().getSceneDecorator().setEnableShadows(True)


    print("Scene built!")
    return root


# Entry point when this script is started with python executable.
def main():
    init_app(
        name=__name__,
        scenes=[(buildScene, "1")],
        autoStepping=True,
    )


if __name__ == "__main__":
    main()





