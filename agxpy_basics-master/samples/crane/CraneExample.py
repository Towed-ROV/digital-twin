import demoutils

import agx

from samples.crane.CraneAssembly import CraneAssembly, CraneController


def build_scene():
    crane = CraneAssembly()
    demoutils.sim().add(crane)
    demoutils.sim().addEventListener(CraneController(crane))

    lock = agx.LockJoint(crane.link1)
    demoutils.sim().add(lock)

