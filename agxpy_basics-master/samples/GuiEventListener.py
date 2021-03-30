
import agxSDK


class MyGuiEventListener(agxSDK.GuiEventListener):

    def __init__(self):
        super().__init__()

    def keyboard(self, key, alt, x, y, down) -> bool:
        handled = False

        # implement logic

        return handled
