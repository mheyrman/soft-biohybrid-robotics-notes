# -*- coding: utf-8 -*-
import Sofa.Core
import Sofa.constants.Key as Key

from stlib3.scene import MainHeader, ContactHeader
from stlib3.physics.rigid import Floor, Cube, RigidObject
from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.constraints import FixedBox
from softrobots.actuators import PullingCable
from stlib3.physics.collision import CollisionMesh
from splib3.loaders import loadPointListFromFile
from gripper import Gripper
import os
import numpy as np

# Finger controller to pull cable with keyboard from cable-gripper tutorial and Task3
class FingerController(Sofa.Core.Controller):
      def __init__(self, *args, **kwargs):
            Sofa.Core.Controller.__init__(self, args, kwargs)
            self.cable = args[0]
            # since there are 3 finger objects on the gripper, name the controllers accordingly
            self.name = "FingerController" + str(args[1])

      # plus function: pulls cable in to grasp objects
      def plus(self):
            displacement = self.cable.CableConstraint.value[0]                # get current cable displacement
            displacement += 1.                                                # iterate by 1 to pull cable in
            stop = False                                                      # boolean stopping condition for state machine

            # check if displacement is greater than 10, at which point the object has been grasped
            if displacement > 10: 
                  stop = True                                                 # if so, stop and continue to next state in state machine
                  displacement = 10                                           # maintain this displacement value
            
            self.cable.CableConstraint.value = [displacement]                 # set cable displacement to new value
            return stop                                                       # return stopping condition

      # minus function: releases cable to let go of objects
      def minus(self):
            displacement = self.cable.CableConstraint.value[0]                # get current cable displacement
            displacement -= 1.                                                # iterate by -1 to release cable
            stop = False                                                      # boolean stopping condition for state machine

            # check if displacement is less than 0, at which point the object has been released
            if displacement < 0:
                  stop = True                                                 # if so, stop and continue to next state in state machine
                  displacement = 0                                            # maintain this displacement value
            
            self.cable.CableConstraint.value = [displacement]                 # set cable displacement to new value
            return stop                                                       # return stopping condition

# Gripper controller that contains gripper state machine to move to and grasp objects
class GripperController(Sofa.Core.Controller):
      def __init__(self, *args, **kwargs):
            Sofa.Core.Controller.__init__(self, args, kwargs)
            self.gripper = args[0]
            self.name = "GripperController"

            self.move_state = 0           # move to first object
            self.pickup_state = 7         # initial pickup_state = final state of when object is released
            self.pos = [0.0, -100.0, 0.0] # same as translation

      # event listener acts as main loop for controller
      def onEvent(self, params):
            # verify whether the current event is an animation step as those contain 
            # time step information and if the gripper has been properly initialized
            if self.gripper is not None and params['type'] == 'AnimateBeginEvent':
                  self.move()                                                 # move gripper to object and grasp it

      # gripper state machine
      def move(self):
            # print(self.pos)
            if self.move_state == 0:                                          # move to first object
                  if self.pickup_state == 7:                                  # if object is released
                        
                        # check position and iterate until above object
                        if self.pos[0] > 0.0:                           
                              direction = [-1.0, 0.0, 0.0]
                              self.pos[0] -= 1.0
                              self.updatePos(direction)
                        elif self.pos[0] < 0.0:
                              direction = [1.0, 0.0, 0.0]
                              self.pos[0] += 1.0
                              self.updatePos(direction)
                        elif self.pos[1] > -172.0:
                              direction = [0.0, -1.0, 0.0]
                              self.pos[1] -= 1.0
                              self.updatePos(direction)
                        elif self.pos[1] < -172.0:
                              direction = [0.0, 1.0, 0.0]
                              self.pos[1] += 1.0
                              self.updatePos(direction)
                        else:                                                 # when above object, move to pickup state machine
                              self.pickup_state = 0                           # set pickup state to grasping
                              self.pickup()                                   # pickup
                  else:                                     # while object is being grasped, continue to pickup state machine
                        self.pickup()

            if self.move_state == 1:                                          # move to second object
                  if self.pickup_state == 7:                                  # if previous object is released

                        # check position and iterate until above object
                        if self.pos[0] > -200.0:
                              direction = [-1.0, 0.0, 0.0]
                              self.pos[0] -= 1.0
                              self.updatePos(direction)
                        elif self.pos[0] < -200.0:
                              direction = [1.0, 0.0, 0.0]
                              self.pos[0] += 1.0
                              self.updatePos(direction)
                        elif self.pos[1] > -172.0:
                              direction = [0.0, -1.0, 0.0]
                              self.pos[1] -= 1.0
                              self.updatePos(direction)
                        elif self.pos[1] < -172.0:
                              direction = [0.0, 1.0, 0.0]
                              self.pos[1] += 1.0
                              self.updatePos(direction)
                        else:                                                 # when above object, move to pickup state machine
                              self.pickup_state = 0                           # set pickup state to grasping
                              self.pickup()                                   # pickup   
                  else:                                     # while object is being grasped, continue to pickup state machine
                        self.pickup()

            if self.move_state == 2:                                          # move to third object
                  if self.pickup_state == 7:                                  # if previous object is released

                        # check position and iterate until above object
                        if self.pos[0] > 140.0:
                              direction = [-1.0, 0.0, 0.0]
                              self.pos[0] -= 1.0
                              self.updatePos(direction)
                        elif self.pos[0] < 140.0:
                              direction = [1.0, 0.0, 0.0]
                              self.pos[0] += 1.0
                              self.updatePos(direction)
                        elif self.pos[1] > -172.0:
                              direction = [0.0, -1.0, 0.0]
                              self.pos[1] -= 1.0
                              self.updatePos(direction)
                        elif self.pos[1] < -172.0:
                              direction = [0.0, 1.0, 0.0]
                              self.pos[1] += 1.0
                              self.updatePos(direction)
                        else:                                                 # when above object, move to pickup state machine
                              self.pickup_state = 0                           # set pickup state to grasping
                              self.pickup()                                   # pickup
                  else:                                     # while object is being grasped, continue to pickup state machine
                        self.pickup()

      # pickup state machine for grasping, moving, and placing objects
      def pickup(self):
            if self.pickup_state == 0:                            # grab object
                  # use FingerController.plus() to pull cables in
                  stop1 = self.gripper.getChild("ElasticMaterialObject").getObject("FingerController1").plus()
                  stop2 = self.gripper.getChild("ElasticMaterialObject").getObject("FingerController2").plus()
                  stop3 = self.gripper.getChild("ElasticMaterialObject").getObject("FingerController3").plus()

                  # if any of the cables have reached their maximum displacement, stop and move to next state
                  if stop1 or stop2 or stop3:
                        self.pickup_state = 1
            elif self.pickup_state == 1:                          # move up
                  if self.pos[1] < -100.0:
                        direction = [0.0, 1.0, 0.0]
                        self.pos[1] += 1.0
                        self.updatePos(direction)
                  elif self.pos[1] > -100.0:
                        direction = [0.0, -1.0, 0.0]
                        self.pos[1] -= 1.0
                        self.updatePos(direction)
                  else:                                           # when at desired height, move to next state
                        self.pickup_state = 2
            elif self.pickup_state == 2:                          # move to the side
                  if self.pos[2] < -100.0:
                        direction = [0.0, 0.0, 1.0]
                        self.pos[2] += 1.0
                        self.updatePos(direction)
                  elif self.pos[2] > -100.0:
                        direction = [0.0, 0.0, -1.0]
                        self.pos[2] -= 1.0
                        self.updatePos(direction)
                  else:                                           # when at desired position, move to next state
                        self.pickup_state = 3
            elif self.pickup_state == 3:                          # move down
                  if self.pos[1] < -172.0:
                        direction = [0.0, 1.0, 0.0]
                        self.pos[1] += 1.0
                        self.updatePos(direction)
                  elif self.pos[1] > -172.0:
                        direction = [0.0, -1.0, 0.0]
                        self.pos[1] -= 1.0
                        self.updatePos(direction)
                  else:                                           # when at desired placing height, move to next state
                        self.pickup_state = 4
            elif self.pickup_state == 4:                          # release object
                  # use FingerController.minus() to release cables
                  stop1 = self.gripper.getChild("ElasticMaterialObject").getObject("FingerController1").minus()
                  stop2 = self.gripper.getChild("ElasticMaterialObject").getObject("FingerController2").minus()
                  stop3 = self.gripper.getChild("ElasticMaterialObject").getObject("FingerController3").minus()

                  # if any of the cables have been fully released, stop and move to next state
                  if stop1 or stop2 or stop3:
                        self.pickup_state = 5
            elif self.pickup_state == 5:                          # move back up
                  if self.pos[1] < -100.0:
                        direction = [0.0, 1.0, 0.0]
                        self.pos[1] += 1.0
                        self.updatePos(direction)
                  elif self.pos[1] > -100.0:
                        direction = [0.0, -1.0, 0.0]
                        self.pos[1] -= 1.0
                        self.updatePos(direction)
                  else:                                           # when at desired height, move to next state
                        self.pickup_state = 6
            elif self.pickup_state == 6:                          # move back to the side
                  if self.pos[2] < 0.0:
                        direction = [0.0, 0.0, 1.0]
                        self.pos[2] += 1.0
                        self.updatePos(direction)
                  elif self.pos[2] > 0.0:
                        direction = [0.0, 0.0, -1.0]
                        self.pos[2] -= 1.0
                        self.updatePos(direction)
                  else:                                           # when at desired position, move to released state
                        self.pickup_state = 7
                        self.move_state = self.move_state + 1     # iterate moving state to go to next object

      # update position of gripper
      def updatePos(self, direction):
            # using gripper ElasticMaterialObject dofs positions
            eobject = self.gripper.getChild("ElasticMaterialObject")
            mecaobject = eobject.getObject("dofs")

            pos = []                                                    # new position of gripper
            for point in mecaobject.rest_position.value:                # iterate through all points in gripper
                  # add direction of movement to each point to move entire gripper
                  pos.append([point[0] + direction[0], point[1] + direction[1], point[2] + direction[2]])

            mecaobject.findData('rest_position').value = pos            # set new position of gripper

            # update pull points of cables in the same way as above:
            cable1 = eobject.getChild("PullingCable1").getObject("CableConstraint")
            p = cable1.pullPoint.value
            cable1.findData("pullPoint").value = [p[0] + direction[0], p[1] + direction[1], p[2] + direction[2]]

            cable2 = eobject.getChild("PullingCable2").getObject("CableConstraint")
            p = cable2.pullPoint.value
            cable2.findData("pullPoint").value = [p[0] + direction[0], p[1] + direction[1], p[2] + direction[2]
            ]
            cable3 = eobject.getChild("PullingCable3").getObject("CableConstraint")
            p = cable3.pullPoint.value
            cable3.findData("pullPoint").value = [p[0] + direction[0], p[1] + direction[1], p[2] + direction[2]]

# Create 3 finger gripper facing concentrically inwards
def Gripper(parentNode=None):
      selfNode = parentNode.addChild("Gripper")                                           # define gripper node

      translation = [0.0, -100.0, 0.0]                                                    # translation: x, z, y
      rotation=[-90.0, 0.0, 0.0]                                                          # rotation: x, z, y
      fixingBox=[-50.0, 50.0, -100.0, 100.0, 20.0, 100.0]                                 # fixing box: x_min, x_max, y_min, y_max, z_min, z_max 

      gripper = selfNode.addChild("Gripper_GMSH")                                         # define gripper child node
      # import gripper mesh and define material properties
      eobject = ElasticMaterialObject(gripper,
                                    volumeMeshFileName="data/mesh/z_palm_10.vtk",
                                    poissonRatio=0.3,
                                    youngModulus=18000,
                                    totalMass=2.0,
                                    surfaceColor=[0.0, 0.8, 0.7, 1.0],
                                    surfaceMeshFileName="data/mesh/z_palm_10.stl",
                                    rotation=rotation,
                                    translation=translation)
      gripper.addChild(eobject)                                                           # add gripper elastic material object to gripper child node

      FixedBox(eobject, atPositions=fixingBox, doVisualization=True)                      # add fixing box to gripper elastic material object (and visualize for debugging)

      # add 3 pulling cables to gripper assigned to each finger
      cable1 = PullingCable(eobject,
                        "PullingCable1",
                        pullPointLocation=[0, 10, 30],
                        rotation=[-90, 0, 90],
                        translation=[7.0, 8.5, 62], 
                        cableGeometry=loadPointListFromFile("data/mesh/cable.json"));

      eobject.addObject(FingerController(cable1, 1))

      cable2 = PullingCable(eobject,
                        "PullingCable2",
                        pullPointLocation=[-20, 10, -10],
                        rotation=[-30, 0, 90],
                        translation=[-27.0, 8.5, -19.0], 
                        cableGeometry=loadPointListFromFile("data/mesh/cable.json"));

      eobject.addObject(FingerController(cable2, 2))

      cable3 = PullingCable(eobject,
                        "PullingCable3",
                        pullPointLocation=[20, 10, -10],
                        rotation=[30, 0, 90],
                        translation=[45.0, 8.5, -40.0], 
                        cableGeometry=loadPointListFromFile("data/mesh/cable.json"));

      eobject.addObject(FingerController(cable3, 3))

      # add collision mesh to gripper
      CollisionMesh(eobject, name="CollisionMesh",
                        surfaceMeshFileName="data/mesh/z_palm_10.stl",
                        rotation=rotation, translation=translation,
                        collisionGroup=[1, 2])

      selfNode.addObject(GripperController(gripper))

      return selfNode

# Create scene with gripper, floor, and 3 objects
def createScene(rootNode):

      # Defining scene main header and properties, based on tutorial for CableGripper in SoftRobots plugin
      MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots"])
      ContactHeader(rootNode, alarmDistance=4, contactDistance=3, frictionCoef=0.5)       # most materials have a coefficient of friction ~0.2-0.6
      rootNode.VisualStyle.displayFlags = "showCollisionModels"
      # visual mode for debugging:
      # rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')

      # Import all possible required plugins
      rootNode.addObject('RequiredPlugin', pluginName=['SoftRobots SoftRobots.Inverse',
                                                                'SofaMiscCollision',
                                                                'SofaConstraint SofaDeformable',
                                                                'SofaEngine SofaImplicitOdeSolver',
                                                                'SofaLoader SofaOpenglVisual',
                                                                'SofaSimpleFem',
                                                                'SofaSparseSolver',
                                                                'Sofa.Component.AnimationLoop',
                                                                'Sofa.Component.Collision.Geometry',
                                                                'Sofa.Component.Constraint.Lagrangian.Correction',
                                                                'Sofa.Component.Constraint.Projective',
                                                                'Sofa.Component.Engine.Select',
                                                                'Sofa.Component.IO.Mesh',
                                                                'Sofa.Component.LinearSolver.Direct',
                                                                'Sofa.Component.LinearSolver.Iterative',
                                                                'Sofa.Component.Mapping.Linear',
                                                                'Sofa.Component.Mass',
                                                                'Sofa.Component.ODESolver.Backward',
                                                                'Sofa.Component.Setting',
                                                                'Sofa.Component.SolidMechanics.FEM.Elastic',
                                                                'Sofa.Component.StateContainer',
                                                                'Sofa.Component.Topology.Container.Constant',
                                                                'Sofa.GL.Component.Rendering3D',
                                                                'Sofa.Component.SolidMechanics.Spring',
                                                                'Sofa.Component.Visual',
                                                                'Sofa.Component.Collision.Response.Contact',
                                                                'Sofa.Component.Constraint.Lagrangian.Solver',
                                                                'Sofa.Component.Controller',
                                                                'Sofa.Component.Collision.Detection.Algorithm',
                                                                'Sofa.Component.Collision.Detection.Intersection',
                                                                'Sofa.Component.Mapping.NonLinear'])

      # add gripper to rootNode
      Gripper(rootNode)

      # add large floor object to act as main stage for gripper demo
      Floor(rootNode, name="Floor",
            color=[1.0, 0.0, 0.0, 1.0],
            translation=[0.0, -160.0, 0.0],
            uniformScale=10.0,
            isAStaticObject=True)

      # define file path for importing object meshes
      path = os.path.dirname(os.path.abspath(__file__))

      ############################################
      ##                  LEMON                 ##
      ############################################
      mass = 300  # g
      radius = 40 # mm
      I = 2.0 / 5.0 * mass * radius**2                # inertia of a sphere
      lemon = RigidObject(parent = rootNode,
                  name = "Lemon",
                  surfaceMeshFileName= path+'/../014_lemon/poisson/textured.obj',
                  translation=[-200.0, -130.0, 30.0],
                  uniformScale=1000,
                  color=[0.5, 0., 0.5],
                  totalMass=0.3,
                  inertiaMatrix=[I, 0., 0., 0., I, 0., 0., 0., I],
                  volume = 5.,)
      lemon.addObject('UncoupledConstraintCorrection')
      # lemonVisu = lemon.addChild('visu')
      # lemonVisu.addObject('MeshSTLLoader', filename=path+"/../014_lemon/poisson/nontextured.stl", name="loader", scale="1000.0")
      # lemonVisu.addObject('OglModel', src="@loader", template='Vec3', color="0.0 0.7 0.7")
      # lemonVisu.addObject('BarycentricMapping')

      ############################################
      ##                  RUBIK                 ##
      ############################################
      I = 1320 * 575**2 / 12                          # inertia of a cube
      rubik = RigidObject(parent = rootNode,
                  name = "Rubik",
                  surfaceMeshFileName= path+'/../077_rubiks_cube/poisson/textured.obj',
                  translation=[-15.0, -150.0, 10.0],
                  rotation=[90 , 0, 180],
                  uniformScale=1000,
                  color=[1., 1., 0.],
                  totalMass=0.132,
                  inertiaMatrix=[8.0 * I, -3.0 * I, -3.0 * I,
                                    -3.0 * I, 8.0 * I, -3.0 * I,
                                    -3.0 * I, -3.0 * I, 8.0 * I],
                  volume = 57.5**3,)
      rubik.addObject('UncoupledConstraintCorrection')

      ############################################
      ##                 TUNA CAN               ##
      ############################################
      m = 1000 * 0.35 # 350g
      r = 1000 * 0.04
      h = 1000 * 0.03
      I = 1/12 * m * (3 * r**2 + h**2)                # inertia of a cylinder in x and y directions
      Iz = 1/2 * m * r**2                             # inertia of a cylinder in z direction
      print(I)
      tuna_can = RigidObject(parent = rootNode,
                  name = "Can",
                  surfaceMeshFileName= path+'/../007_tuna_fish_can/poisson/textured.obj',
                  translation=[140.0, -120.0, 0.0],
                  rotation=[90, 0, 0],
                  uniformScale=900,
                  color=[1., 1., 0.],
                  totalMass=m / 1000,
                  inertiaMatrix=[I, 0., 0., 0., Iz, 0., 0., 0., I],
                  volume = np.pi * r ** 2 * h,)
      tuna_can.addObject('UncoupledConstraintCorrection')
      
      return rootNode
