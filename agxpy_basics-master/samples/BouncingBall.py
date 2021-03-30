import agx
import agxCollide
from agxRender import Color
from random import random



def build_scene():
    demoutils.register_additional_scenes('build_scene2')
    build_scene1()


def build_scene1():

    # Create a geometry with a plane shape as our 'floor'
    floor_geometry = agxCollide.Geometry(agxCollide.Box(agx.Vec3(10, 10, 0.1)))
    demoutils.create_visual(floor_geometry)
    demoutils.sim().add(floor_geometry)  # Add the geometry to the simulation

    rb1 = agx.RigidBody()  # Create a rigid body
    rb1.add(agxCollide.Geometry(agxCollide.Sphere(0.5)))  # Add a geometry with a sphere-shape of radius 0.5
    rb1.setPosition(0, 0, 5.0)  # Position the sphere somewhere above our plane
    demoutils.create_visual(rb1)
    demoutils.sim().add(rb1)  # Add the body to the simulation. The geometry will also be added


def build_scene2():

    # Create a geometry with a plane shape as our 'floor'
    floor_geometry = agxCollide.Geometry(agxCollide.Box(agx.Vec3(10, 10, 0.1)))
    demoutils.create_visual(floor_geometry, diffuse_color=Color.Green())
    demoutils.sim().add(floor_geometry)  # Add the geometry to the simulation

    for x in range(-5, 5):
        for y in range(-5, 5):
            for z in range(1, 8):
                rb = agx.RigidBody()  # Create a rigid body
                rb.add(agxCollide.Geometry(agxCollide.Sphere(0.2)))  # Add a geometry with a sphere-shape of radius 0.2
                rb.setPosition(x + random(), y + random(), z)  # Position the sphere somewhere above our plane
                demoutils.create_visual(rb)

                demoutils.sim().add(rb)  # Add the body to the simulation. The geometry will also be added
