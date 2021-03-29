import agxSDK
import demoutils


class MyStepEventListener(agxSDK.StepEventListener):

    def __init__(self):
        super().__init__(agxSDK.StepEventListener.POST_STEP)

    def pre(self, t):
        print("pre")  # will not be called as we have specified a POST_STEP mask

    def post(self, t):
        print("post")


def build_scene():
    listener = MyStepEventListener()
    demoutils.sim().addEventListener(listener)
