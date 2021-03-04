'''
This utility script takes a serialized agx simulations as argument and converts all the particleEmitters 
to rigidBodyEmitters. A default (or specified) shape library is used for rigidBody templates. The
bodies match the particle distributionTable on size and material properties.

All the rigidBodies named "Sinki" for i=0...n, are turned to rigidBodySinks.

The new simulation is saved as output.agx by default. It is possible to specify name with the command line 
argument --output <filename.agx>.
'''
import os
import sys
import math
import argparse
import numpy as np

import agx
import agxSDK
import agxUtil
import agxCollide
import agxControl

def inertia_tensor_size_estimation(mp):
    ''' Returns the an approximation of a rock size given its diagonalized inertia tensor '''
    m = mp.getMass()
    it = mp.getInertiaTensor()
    np_it = np.zeros((3,3))
    for i in range(3):
        for j in range(3):
            np_it[i,j] = it.at(i,j)
    
    (w,v) = np.linalg.eig(np_it)
    
    Ixx = w[0]
    Iyy = w[1]
    Izz = w[2]
    x2 = 6/m * (Izz + Iyy - Ixx)
    y2 = 6/m * (Izz + Ixx - Iyy)
    z2 = 6/m * (Iyy + Ixx - Izz)
    
    # Find the middle coordinate
    size = 0
    if x2 > y2:
        if x2 < z2:
            size = math.sqrt(x2)
        elif z2 < y2:
            size = math.sqrt(y2)
        else:
            size = math.sqrt(z2)
    else:
        if y2 < z2:
            size = math.sqrt(y2)
        elif z2 < x2:
            size = math.sqrt(x2)
        else:
            size = math.sqrt(z2)
    return size

def create_rigid_body_from_obj(name, scale, material):
    convexShapes = agxCollide.ConvexRefVector()
    convexBuilder = None

    agxUtil.createCdFromWavefrontOBJ(name, convexShapes, convexBuilder, scale)
    rb = agx.RigidBody()
    for c in convexShapes:
        assert(c)
        g = agxCollide.Geometry(c.asConvex())
        if material:
            g.setMaterial(material)
        rb.add(g)
    return rb

def create_distribution_table(shape_paths, sizes, weights, material):
    dt = agx.EmitterDistributionTable()    
    n = len(shape_paths) * len(sizes)
    
    for shape_path in shape_paths:
        # Get the original size of the model
        rb_original = create_rigid_body_from_obj(shape_path, agx.Matrix3x3(), None)
        size_original = inertia_tensor_size_estimation(rb_original.getMassProperties())
        # add a correctly scaled rb to the distribution table for each size
        for s, w in zip(sizes, weights):
            scale = agx.Matrix3x3() * s / size_original
            rb = create_rigid_body_from_obj(shape_path, scale, material)
            model = agx.RigidBodyEmitterDistributionModel(rb, w/n)
            dt.addModel(model)
    return dt

def find_shapes(path):
    shapes = []
    print("Finding shapes...")
    for f in os.listdir(path):
        file_extension = os.path.splitext(f)[1]
        if ".obj" == file_extension:
            print("... adding shape %s to shape library" % f)
            shapes.append(os.path.join(path, f))
    return shapes

def create_rigid_body_material_from_particle_material(pmat):
    mat = agx.Material("rb_%s" % pmat.getName())
    mat.getBulkMaterial().setDensity(pmat.getBulkMaterial().getDensity())
    # mat.getBulkMaterial().setYoungsModulus(pmat.getBulkMaterial().getYoungsModulus())
    # mat.getBulkMaterial().setPoissonsRatio(pmat.getBulkMaterial().getPoissonsRatio())
    return mat

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", type=str, help=".agx file whos particleEmitters will be replaced with rigidBodyEmitters.")
    ap.add_argument("-o", "--output", default="output.agx", type=str, help="Path for the resulting .agx file.")
    ap.add_argument("--shapeLibrary", type=str, default="", help="Path to the shape library.")
    args = vars(ap.parse_args())

    input_path = args["input"]
    output_path = args["output"]
    shape_library_path = args["shapeLibrary"]

    if os.path.isfile(input_path):
        print("The serialized simulation to load:", input_path)
    else:
        sys.exit("Could not find input path:", input_path )
    print("The path where the new simulation is saved:", os.path.abspath(output_path))

    # check that the shape library path exists
    if not shape_library_path:
        if os.environ.get("AGX_DATA_DIR"):
            shape_library_path = os.path.join(os.environ.get("AGX_DATA_DIR"), "models\\convex_stones")
        else:
            sys.exit("No shape library found or given.")

    if os.path.isdir(shape_library_path):
        print("The path to the shape library is:", shape_library_path)
    else:
        sys.exit("Shape library path do not exist, or is not a directory:", shape_library_path)
    print("------------------------------------------------------------------------------")
    shape_paths = find_shapes(shape_library_path)
    if len(shape_paths) < 1:
        sys.exit("Did not find any shapes in ", shape_library_path)

    sim = agxSDK.Simulation()
    sim.read(input_path, None, agxSDK.Simulation.READ_NONE)

    print("------------------------------------------------------------------------------")
    particle_emitters = sim.getParticleEmitters()
    if particle_emitters is None:
        sys.exit("Did not find any particle emitters in simulation.")
    else:
        print("Found %d particle emitters. Replacing with rigidbody emitters ..." % len(particle_emitters))
    
    existing_materials = {}
    geoms = sim.getGeometries()
    for g in geoms:
        m = g.getMaterial()
        if m not in existing_materials.values():
            existing_materials[m.getName()] = m

    # Replace each of the particle emitters with rigid body emitters
    particle_model_materials = []
    rb_model_materials = []
    for pe in particle_emitters:
        dt = pe.getDistributionTable()
        # get emitter geometry
        geom = pe.getGeometry()
        geom.setSensor(True)
        # get the size, material and probability weight of all particle models
        weights = []
        sizes = []
        models = dt.getModels()
        for pm in models:
            m = pm.asParticleEmitterDistributionModel()
            sizes.append(m.getParticleRadius() * 2.0)
            weights.append(m.getProbabilityWeight())
            material = m.getParticleMaterial()
        
        if len(sizes) < 1:
            sys.exit("No particle models set on particle emitter", geom.getName())
        
        # create a new material to use for the rigidbodies
        particle_model_materials.append(material)
        rb_mat = create_rigid_body_material_from_particle_material(material)
        rb_model_materials.append(rb_mat)
        # create rigid body distribution table from shape library
        dt_rb = create_distribution_table(shape_paths, sizes, weights, rb_mat)
        dt_rb.setProbabilityQuantity(dt.getProbabilityQuantity())

        # create rigidbody emitter
        rbe = agx.RigidBodyEmitter()
        rbe.setQuantity(pe.getQuantity())
        rbe.setRate(pe.getRate())
        rbe.setMaximumEmittedQuantity(pe.getMaximumEmittedQuantity())
        rbe.setGeometry(geom)
        rbe.setDistributionTable(dt_rb)
        
        # switch the emitter type in the simulation
        sim.add(rbe)
        sim.remove(pe)
        print("Replaced particle emitter on geometry: %s" % geom.getName())


    print("------------------------------------------------------------------------------")
    print("Copying contact material properties from old particle material to the new rb material ")
    mm = sim.getMaterialManager()
    iterative_projected_cone_friction = agx.IterativeProjectedConeFriction()
    iterative_projected_cone_friction.setSolveType(agx.FrictionModel.ITERATIVE)
    # Copy material properties from the old particle material to the new rigidbody material
    def copy_cm_properties(old_pair, new_pair):
        cm0 = mm.getContactMaterial(old_pair[0], old_pair[1])
        restitution = cm0.getRestitution()
        friction = cm0.getFrictionCoefficient()
        cm1 = mm.getOrCreateContactMaterial(new_pair[0], new_pair[1])
        cm1.setRestitution(restitution)
        cm1.setFrictionCoefficient(friction)
        return cm1
    i = 0
    for particle_material, rb_material in zip(particle_model_materials, rb_model_materials):
        for k,v in existing_materials.items():
            cm1 = copy_cm_properties((particle_material, v), (v, rb_material))
            print("Material properties for contact material: %s-%s" % (k, rb_material.getName()))
            print("... friction coefficient is: ", cm1.getFrictionCoefficient())
            print("... restitution is: ", cm1.getRestitution())
        for particle_material2, rb_material2 in zip(particle_model_materials[i:], rb_model_materials[i:]):
            cm1 = copy_cm_properties((particle_material, particle_material2), (rb_material, rb_material2))
            cm1.setFrictionModel(iterative_projected_cone_friction)
            print("Material properties for contact material: %s-%s" % (rb_material.getName(), rb_material2.getName()))
            print("... friction coefficient is: ", cm1.getFrictionCoefficient())
            print("... restitution is: ", cm1.getRestitution())
        i += 1

    print("------------------------------------------------------------------------------")
    print("Removing all the particle materials...")
    # Remove all the old particle materials
    for pmat in particle_model_materials:
        print("... removing material \'%s\'" % pmat.getName())
        sim.getMaterialManager().remove(pmat)

    print("------------------------------------------------------------------------------")
    print("Finding particle sinks and making them rigidBody sinks...")
    # Find all the rigidbodies that are named "Sinki" for i=0...n where n+1 returns None.
    # and set a sensoroperation to remove rigidBodies.
    i = 0
    while True:
        rb = sim.getRigidBody("Sink%d" % i)
        if rb is not None:
            for g in rb.getGeometries():
                if g.isSensor():
                    print("... Found sink geometry \'%s\'. Is now rigidBody sink aswell." % g.getName())
                    event_sensor = agxControl.EventSensor(g)
                    sensor_op = agxControl.RemoveRigidBody()
                    event_sensor.addSensorEvent(sensor_op)
                    sim.add(event_sensor)
        else:
            break
        i += 1

    print("------------------------------------------------------------------------------")
    sim.write(os.path.abspath(output_path))
    print("Saved simulation as:", os.path.abspath(output_path))
    print("You can run the simulation with the command agxviewer %s" % os.path.abspath(output_path))
    print("Use the flags --journalRecord --journalIncrementalStructure to record a journal.")

if __name__ == '__main__':
    init = agx.AutoInit()
    main()