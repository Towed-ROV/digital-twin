import demoutils

import agx
import agxCollide

from agxRender import Color


def create_box(position: agx.Vec3):
    shape = agxCollide.Box(agx.Vec3(0.5, 0.5, 0.5))
    geometry = agxCollide.Geometry(shape)
    body = agx.RigidBody(geometry)
    body.setPosition(position)
    demoutils.create_visual(body)
    return body


def create_sphere(position: agx.Vec3):
    shape = agxCollide.Sphere(0.5)
    geometry = agxCollide.Geometry(shape)
    body = agx.RigidBody(geometry)
    body.setPosition(position)
    demoutils.create_visual(body)
    return body


def create_capsule(position: agx.Vec3):
    shape = agxCollide.Capsule(0.5, 1)
    geometry = agxCollide.Geometry(shape)
    body = agx.RigidBody(geometry)
    body.setPosition(position)
    demoutils.create_visual(body)
    body.setRotation(agx.EulerAngles(agx.PI_2, 0, 0))
    return body


def create_cylinder(position: agx.Vec3):
    shape = agxCollide.Cylinder(0.5, 1)
    geometry = agxCollide.Geometry(shape)
    body = agx.RigidBody(geometry)
    body.setPosition(position)
    demoutils.create_visual(body)
    body.setRotation(agx.EulerAngles(agx.PI_2, 0, 0))
    return body


def build_scene():

    sim = demoutils.sim()

    # Create a geometry with a plane shape as our 'floor'
    floor_geometry = agxCollide.Geometry(agxCollide.Box(agx.Vec3(10, 10, 0.1)))
    demoutils.create_visual(floor_geometry, Color.Green())
    sim.add(floor_geometry)  # Add the geometry to the simulation

    sim.add(create_box(agx.Vec3(-2, 3, 5)))
    sim.add(create_sphere(agx.Vec3(0, 1, 5)))
    sim.add(create_capsule(agx.Vec3(3, 5, 5)))
    sim.add(create_cylinder(agx.Vec3(-3, -2, 5)))
