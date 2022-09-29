# Imports
import numpy as np
import os
import sys
import warnings

from pydrake.examples.manipulation_station import ManipulationStation
from pydrake.geometry import MeshcatVisualizerCpp, MeshcatVisualizerParams, StartMeshcat, RoleAssign

from pydrake.systems.analysis import Simulator
import pydrake.geometry as drakegeometry

from pydrake.all import (
    AbstractValue, Adder, AddMultibodyPlantSceneGraph, BallRpyJoint, BaseField,
    Box, CameraInfo, ClippingRange, CoulombFriction, Cylinder, Demultiplexer,
    DiagramBuilder, DepthRange, DepthImageToPointCloud, DepthRenderCamera, AddCompliantHydroelasticProperties, 
    FindResourceOrThrow, GeometryInstance, InverseDynamicsController, ScrewJoint, AddRigidHydroelasticProperties,
    LeafSystem, MakeMultibodyStateToWsgStateSystem, LogVectorOutput, MultibodyPlantConfig,  
    MakePhongIllustrationProperties, MakeRenderEngineVtk, MakeRenderEngineGl, ModelInstanceIndex,
    MultibodyPlant, Parser, PassThrough, PrismaticJoint, RenderCameraCore, SnoptSolver, AddMultibodyPlant,
    RenderEngineVtkParams, RevoluteJoint, Rgba, RigidTransform, RollPitchYaw, PositionConstraint, OrientationConstraint,
    RotationMatrix, RgbdSensor, SchunkWsgPositionController, SpatialInertia, DifferentialInverseKinematicsStatus,
    Sphere, StateInterpolatorWithDiscreteDerivative, UnitInertia, DifferentialInverseKinematicsParameters, 
    DifferentialInverseKinematicsIntegrator, Role, SpatialVelocity, PiecewisePose, JacobianWrtVariable, RandomGenerator,
    ContactVisualizerParams, ContactVisualizer, InverseKinematics, Solve, DoDifferentialInverseKinematics, ReadObjToTriangleSurfaceMesh,
    SpatialForce, ExternallyAppliedSpatialForce, BasicVector, Value, List, IpoptSolver, MultibodyForces, MathematicalProgram)
import warnings
warnings.simplefilter("ignore")




def AddWrenchObject(builder, plant, scene_graph, tool_name, instance_info={}):
    """
    A way to handle the screw joint special case
    """
    parser = Parser(plant)
    obj_path = 'wrench' # objects_sdf
    object_paths = [os.path.join(obj_path, tool_name, o, o + '.sdf') for o in os.listdir(os.path.join(obj_path, tool_name))]
    idx = np.random.randint(len(object_paths)) 
    path = object_paths[idx]
    if len(instance_info) > 0:
        path = [p for p in object_paths if instance_info['object_name'].split("_")[1] in p][0]  

    model_name = path.split('/')[-1][:-4]
    object_name = model_name + '_body_link'
    tool_obj = parser.AddModelFromFile(path, object_name)

    # build the bolt-nut combo child rotates and translates on the axis of Z of the parent
    bolt = tool_obj
    nut_name = object_name.replace("_body", '_nut_body')
    nut_path = path.replace(".sdf", '_nut.sdf')

    nut = parser.AddModelFromFile(nut_path, nut_name)
    nut_body_id = plant.GetBodyIndices(nut)[0]
    nut_body_frame_id = plant.GetBodyFrameIdOrThrow(nut_body_id)
    bolt_body_frame_id = plant.GetBodyFrameIdOrThrow(plant.GetBodyIndices(bolt)[0])

    # body_frame
    nut_body_frame  = plant.GetBodyFromFrameId(nut_body_frame_id).body_frame()
    bolt_body_frame = plant.GetBodyFromFrameId(bolt_body_frame_id).body_frame()
    screw_joint = ScrewJoint("nut_bolt_joint", bolt_body_frame, nut_body_frame, screw_pitch=0.005, damping=1e2)

    # weld the bolt to the table    
    plant.AddJoint(screw_joint)
    bolt_pose = RigidTransform(RollPitchYaw(0, 0, 0), [0.46, 0, 0.13])
    plant.WeldFrames(plant.world_frame(), bolt_body_frame, bolt_pose)
    tool_obj = nut

builder = DiagramBuilder()


multibody_plant_config =  MultibodyPlantConfig( time_step=0.001,
                                                contact_surface_representation="polygon" ,
                                                contact_model="hydroelastic_with_fallback",
                                                discrete_contact_solver="sap" )
plant, scene_graph = AddMultibodyPlant(multibody_plant_config, builder) 
AddWrenchObject(builder, plant, scene_graph, '.')

plant.Finalize()
builder.ExportOutput(plant.get_contact_results_output_port(), "contact_results")

meshcat = StartMeshcat()
print("meshcat: ", meshcat.web_url())
visualizer = MeshcatVisualizerCpp.AddToBuilder(builder, scene_graph.get_query_output_port(), meshcat, MeshcatVisualizerParams())
diagram = builder.Build()

# extra_force_system.wrench = np.concatenate((100 * np.ones(3), 100 * np.ones(3)))
simulator = Simulator(diagram)
simulator.Initialize()
context = simulator.get_context()
# contact_measurement_port = diagram.GetOutputPort("contact_results")

context.SetTime(0)
# contact_results = contact_measurement_port.Eval(context)
simulator.AdvanceTo(np.inf)
