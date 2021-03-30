
import agx
import agxCollide

init = agx.AutoInit()

shape = agxCollide.Sphere(0.5)
geometry = agxCollide.Geometry(shape)
geometry.setEnableCollisions(False)  # Geometry will not collide with other objects
body = agx.RigidBody(geometry)
body.setMotionControl(agx.RigidBody.DYNAMICS)  # Default value

