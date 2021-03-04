
try:
    from PySide2 import QtCore, QtGui, QtWidgets, QtOpenGL
    from PySide2.QtCore import Qt
    from PySide2.QtCharts import QtCharts
except ImportError:
    print("\n*** Missing required PySide2 package.")
    print("*** Run: pip install pyside2\n")


import osg
import agx
import agxSDK
import agxPython
import agxOSG

import math
import sys

try:
    from OpenGL.GL import *
except ImportError:
    app = QtWidgets.QApplication(sys.argv)
    messageBox = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Critical, "AGX QT Sample",
                                    "PyOpenGL must be installed to run this example.",
                                    QtWidgets.QMessageBox.Close)
    messageBox.setDetailedText("Run:\npip install PyOpenGL PyOpenGL_accelerate")
    messageBox.exec_()
    sys.exit(1)


class GLWidget(QtOpenGL.QGLWidget):

    """Integrates PySide2 QGLWindow with an osg based rendering

    
    Examples:
        class MyGLWidget(GLWidget):
            def __init__(self, parent=None):
                super(MyGLWidget, self).__init__(parent)
        
                #Change the update rate of the update method
                self.getUpdateTimer().setInterval(10)

            

    """


    def __init__(self, parent=None, update_interval=10):
        super(GLWidget, self).__init__(parent)

        self.setFocusPolicy(Qt.StrongFocus)
        self.m_update_timer = QtCore.QTimer(self)
        self.m_update_timer.timeout.connect(self.update)
        self.m_update_timer.start(update_interval)
        self.initOSG()

    def getUpdateTimer(self):
        return self.m_update_timer

    def initOSG(self):
        self.embedded_window = agxOSG.EmbeddedGLWindow()
        assert(self.embedded_window.init(self.x(), self.y(), self.height(), self.width() ))

    def getEmbeddedWindow(self):
        return self.embedded_window

    def getRoot(self):
        return self.embedded_window.getSceneDecorator()

    '''
    def buildScene(self):
        self.sim = agxSDK.Simulation()
        self.sim.setTimeStep(1/100)
        self.embedded_window.setSimulation(self.sim)
        self.sim.getRenderManager().setEnable( False )
        self.getRoot().setEnable(True)
        self.getRoot().setEnableLogo(False)
        self.getRoot().setEnableShadows(True)
        self.getRoot().setShadowMethod(agxOSG.SceneDecorator.SOFT_SHADOWMAP)

        self.getRoot().setEnableShaderState(False)
        self.getRoot().setText(3, "Hej" )

        ground = agxCollide.Geometry( agxCollide.Box( 10, 10, 0.5 ) )
        ground.setName("Ground")
        ground.setPosition( 0, 0, -0.5 )
        self.sim.add( ground )

        node = agxOSG.createVisual( ground, self.getRoot() )
        agxOSG.setDiffuseColor( node, agxRender.Color.Green() )

        body = None

        for i in range(10):
            box = agxCollide.Geometry( agxCollide.Box( 0.3, 0.3, 0.3 ) )
            box.setName("Box_{}".format(i))
            b = agx.RigidBody(box)
            body = b
            b.setName("Box_{}".format(i))
            self.sim.add(b)
            node = agxOSG.createVisual(box, self.getRoot())
            agxOSG.setDiffuseColor(node, agxRender.Color.Green())
            b.setPosition(i,0,2)

        self.getRoot().setBackgroundColor( agxRender.Color.SkyBlue() , agxRender.Color.DodgerBlue())
        
        self.embedded_window.fitSceneIntoView()
        self.getRoot().calculateLightPositions(self.getRoot())

        self.getRoot().getLightSource(agxOSG.SceneDecorator.LIGHT0).setPosition(agx.Vec4(10,10,10,1))
        self.getRoot().getLightSource(agxOSG.SceneDecorator.LIGHT1).setPosition(agx.Vec4(-10,10,10,1))
        self.getRoot().getLightSource(agxOSG.SceneDecorator.LIGHT2).setPosition(agx.Vec4(10,19,10,1))
    '''

    def clear(self):
        self.embedded_window.clear()
        
    def closeEvent(self,event):
        self.clear()

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)    
        self.embedded_window.frame()

    def resizeGL(self, width, height):
        side = min(width, height)
        if side < 0:
            return

        self.embedded_window.resize(self.x(), self.y(), width, height)


    @classmethod
    def qtToOSGMouseButton(self, button):
        osg_button = 0
        if (button == QtCore.Qt.LeftButton):
            osg_button = 1
        elif (button == Qt.MiddleButton):
            osg_button = 2
        elif (button == Qt.RightButton):
            osg_button = 3
        return osg_button

    def wheelEvent(self, event):
        delta = event.delta()
        motion = 0
        
        if (delta > 0):
            motion = 3 #agxSDK.GuiEventListener.SCROLL_UP
        else:
            motion = 4 #agxSDK.GuiEventListener.SCROLL_DOWN
        self.embedded_window.wheelEvent(motion)


    def mousePressEvent(self, event):
        button = self.qtToOSGMouseButton(event.button())
        self.embedded_window.mousePressEvent(event.x(), event.y(), button)

    def mouseReleaseEvent(self, event):
        button = self.qtToOSGMouseButton(event.button())
        self.embedded_window.mouseReleaseEvent(event.x(), event.y(), button)

    def mouseMoveEvent(self, event):
        x = event.x()
        y = event.y()
        self.embedded_window.mouseMoveEvent(x,y)

    @classmethod
    def getOSGKeyboardModifierMask(self, event):
        modkey = event.modifiers() & (Qt.ShiftModifier | Qt.ControlModifier | Qt.AltModifier)
        mask = 0
        if ( modkey & Qt.ShiftModifier ):
            mask |= agxSDK.GuiEventListener.MODKEY_SHIFT
        if ( modkey & Qt.ControlModifier ):
            mask |= agxSDK.GuiEventListener.MODKEY_CTRL
        if ( modkey & Qt.AltModifier ):
            mask |= agxSDK.GuiEventListener.MODKEY_ALT
        return mask


    def keyPressEvent(self, event):
        osg_key = event.key()
        mod_mask = self.getOSGKeyboardModifierMask(event)
        
        if osg_key in agxSDK.KeyboardMap:
           osg_key = agxSDK.KeyboardMap[osg_key]

        self.embedded_window.keyPressEvent(osg_key, mod_mask)

    def keyReleaseEvent(self, event):

        if (event.isAutoRepeat()):
            event.ignore()
            return

        osg_key = event.key()

        mod_mask = self.getOSGKeyboardModifierMask(event)

        if osg_key in agxSDK.KeyboardMap:
           osg_key = agxSDK.KeyboardMap[osg_key]

        self.embedded_window.keyReleaseEvent(osg_key, mod_mask)

    def update(self):
        self.updateGL()


agxSDK.KeyboardMap = { 
    Qt.Key_Shift: agxSDK.GuiEventListener.KEY_Shift_L,
    Qt.Key_Shift: agxSDK.GuiEventListener.KEY_Shift_R,
    Qt.Key_Control: agxSDK.GuiEventListener.KEY_Control_L,
    Qt.Key_Control: agxSDK.GuiEventListener.KEY_Control_R,
    Qt.Key_Alt: agxSDK.GuiEventListener.KEY_Alt_L, # Qt doesn't have a Alt L
    Qt.Key_Alt: agxSDK.GuiEventListener.KEY_Alt_R, # Qt doesn't have a Alt R

    Qt.Key_Backspace : agxSDK.GuiEventListener.KEY_BackSpace,
    Qt.Key_Tab : agxSDK.GuiEventListener.KEY_Tab,
    Qt.Key_Return: agxSDK.GuiEventListener.KEY_Linefeed, # No LineFeed in Qt!
    Qt.Key_Clear: agxSDK.GuiEventListener.KEY_Clear,
    Qt.Key_Return: agxSDK.GuiEventListener.KEY_Return,
    Qt.Key_Pause: agxSDK.GuiEventListener.KEY_Pause,
    Qt.Key_ScrollLock: agxSDK.GuiEventListener.KEY_Scroll_Lock,
    Qt.Key_SysReq: agxSDK.GuiEventListener.KEY_Sys_Req,
    Qt.Key_Escape: agxSDK.GuiEventListener.KEY_Escape,
    Qt.Key_Delete: agxSDK.GuiEventListener.KEY_Delete,

    Qt.Key_Home: agxSDK.GuiEventListener.KEY_Home,
    Qt.Key_Left: agxSDK.GuiEventListener.KEY_Left,
    Qt.Key_Up : agxSDK.GuiEventListener.KEY_Up, 
    Qt.Key_Right: agxSDK.GuiEventListener.KEY_Right, 
    Qt.Key_Down: agxSDK.GuiEventListener.KEY_Down,
    Qt.Key_Left: agxSDK.GuiEventListener.KEY_Prior, # no Prior in Qt
    Qt.Key_PageUp: agxSDK.GuiEventListener.KEY_Page_Up,
    Qt.Key_Right: agxSDK.GuiEventListener.KEY_Next, # No Next in Qt
    Qt.Key_PageDown: agxSDK.GuiEventListener.KEY_Page_Down,
    Qt.Key_End: agxSDK.GuiEventListener.KEY_End,
    Qt.Key_Home: agxSDK.GuiEventListener.KEY_Begin, # No Begin in Qt

    Qt.Key_Select: agxSDK.GuiEventListener.KEY_Select,
    Qt.Key_Print: agxSDK.GuiEventListener.KEY_Print,
    Qt.Key_Execute: agxSDK.GuiEventListener.KEY_Execute,
    Qt.Key_Insert: agxSDK.GuiEventListener.KEY_Insert,
    #: agxSDK.GuiEventListener.KEY_Undo, Qt.Key_ # no Undo
    #: agxSDK.GuiEventListener.KEY_Redo, Qt.Key_ # no Redo
    Qt.Key_Menu: agxSDK.GuiEventListener.KEY_Menu,
    Qt.Key_Search: agxSDK.GuiEventListener.KEY_Find, # no Qt Find
    Qt.Key_Cancel: agxSDK.GuiEventListener.KEY_Cancel,
    Qt.Key_Help: agxSDK.GuiEventListener.KEY_Help,
    Qt.Key_Escape: agxSDK.GuiEventListener.KEY_Break, # no break
    Qt.Key_Mode_switch: agxSDK.GuiEventListener.KEY_Mode_switch,
    Qt.Key_Mode_switch: agxSDK.GuiEventListener.KEY_Script_switch, # no Script switch
    Qt.Key_NumLock: agxSDK.GuiEventListener.KEY_Num_Lock,

    Qt.Key_CapsLock: agxSDK.GuiEventListener.KEY_Caps_Lock,
    Qt.Key_CapsLock: agxSDK.GuiEventListener.KEY_Shift_Lock,

    Qt.Key_Meta : agxSDK.GuiEventListener.KEY_Meta_L,# Qt doesn't have a Meta L
    Qt.Key_Meta: agxSDK.GuiEventListener.KEY_Meta_R,  # Qt doesn't have a Meta R
    Qt.Key_Super_L: agxSDK.GuiEventListener.KEY_Super_L,
    Qt.Key_Super_R: agxSDK.GuiEventListener.KEY_Super_R,
    Qt.Key_Hyper_L: agxSDK.GuiEventListener.KEY_Hyper_L,
    Qt.Key_Hyper_R: agxSDK.GuiEventListener.KEY_Hyper_R,

    Qt.Key_Space: agxSDK.GuiEventListener.KEY_KP_Space,
    Qt.Key_Tab: agxSDK.GuiEventListener.KEY_KP_Tab,
    Qt.Key_Enter: agxSDK.GuiEventListener.KEY_KP_Enter,
    Qt.Key_F1: agxSDK.GuiEventListener.KEY_KP_F1,
    Qt.Key_F2: agxSDK.GuiEventListener.KEY_KP_F2,
    Qt.Key_F3: agxSDK.GuiEventListener.KEY_KP_F3,
    Qt.Key_F4: agxSDK.GuiEventListener.KEY_KP_F4,
    Qt.Key_Home: agxSDK.GuiEventListener.KEY_KP_Home,
    Qt.Key_Left: agxSDK.GuiEventListener.KEY_KP_Left,
    Qt.Key_Up: agxSDK.GuiEventListener.KEY_KP_Up,
    Qt.Key_Right: agxSDK.GuiEventListener.KEY_KP_Right,
    Qt.Key_Down: agxSDK.GuiEventListener.KEY_KP_Down,
    Qt.Key_Left: agxSDK.GuiEventListener.KEY_KP_Prior,
    Qt.Key_PageUp: agxSDK.GuiEventListener.KEY_KP_Page_Up,
    Qt.Key_Right: agxSDK.GuiEventListener.KEY_KP_Next,
    Qt.Key_PageDown: agxSDK.GuiEventListener.KEY_KP_Page_Down,
    Qt.Key_End: agxSDK.GuiEventListener.KEY_KP_End,

    # : agxSDK.GuiEventListener.KEY_KP_Begin, Qt.Key_Begin
    Qt.Key_Insert: agxSDK.GuiEventListener.KEY_KP_Insert,
    Qt.Key_Delete: agxSDK.GuiEventListener.KEY_KP_Delete,
    Qt.Key_Equal: agxSDK.GuiEventListener.KEY_KP_Equal,
    Qt.Key_Asterisk: agxSDK.GuiEventListener.KEY_KP_Multiply,
    Qt.Key_Plus: agxSDK.GuiEventListener.KEY_KP_Add,
    #: agxSDK.GuiEventListener.KEY_KP_Separator, Qt.Key_
    Qt.Key_Minus: agxSDK.GuiEventListener.KEY_KP_Subtract,
    Qt.Key_Period: agxSDK.GuiEventListener.KEY_KP_Decimal,
    Qt.Key_division: agxSDK.GuiEventListener.KEY_KP_Divide,
    Qt.Key_0: agxSDK.GuiEventListener.KEY_KP_0,
    Qt.Key_1: agxSDK.GuiEventListener.KEY_KP_1,
    Qt.Key_2: agxSDK.GuiEventListener.KEY_KP_2,
    Qt.Key_3: agxSDK.GuiEventListener.KEY_KP_3,
    Qt.Key_4: agxSDK.GuiEventListener.KEY_KP_4,
    Qt.Key_5: agxSDK.GuiEventListener.KEY_KP_5,
    Qt.Key_6: agxSDK.GuiEventListener.KEY_KP_6,
    Qt.Key_7: agxSDK.GuiEventListener.KEY_KP_7,
    Qt.Key_8: agxSDK.GuiEventListener.KEY_KP_8,
    Qt.Key_9: agxSDK.GuiEventListener.KEY_KP_9,

    Qt.Key_F1: agxSDK.GuiEventListener.KEY_F1, 
    Qt.Key_F2: agxSDK.GuiEventListener.KEY_F2,
    Qt.Key_F3: agxSDK.GuiEventListener.KEY_F3,
    Qt.Key_F4: agxSDK.GuiEventListener.KEY_F4,
    Qt.Key_F5: agxSDK.GuiEventListener.KEY_F5,
    Qt.Key_F6: agxSDK.GuiEventListener.KEY_F6,
    Qt.Key_F7: agxSDK.GuiEventListener.KEY_F7,
    Qt.Key_F8: agxSDK.GuiEventListener.KEY_F8,
    Qt.Key_F9: agxSDK.GuiEventListener.KEY_F9,
    Qt.Key_F10: agxSDK.GuiEventListener.KEY_F10,
    Qt.Key_F11: agxSDK.GuiEventListener.KEY_F11,
    Qt.Key_F12: agxSDK.GuiEventListener.KEY_F12,
    Qt.Key_F13: agxSDK.GuiEventListener.KEY_F13,
    Qt.Key_F14: agxSDK.GuiEventListener.KEY_F14,
    Qt.Key_F15: agxSDK.GuiEventListener.KEY_F15, 
    Qt.Key_F16: agxSDK.GuiEventListener.KEY_F16, 
    Qt.Key_F17: agxSDK.GuiEventListener.KEY_F17,
    Qt.Key_F18: agxSDK.GuiEventListener.KEY_F18,
    Qt.Key_F19: agxSDK.GuiEventListener.KEY_F19,
    Qt.Key_F20: agxSDK.GuiEventListener.KEY_F20,
    Qt.Key_F21: agxSDK.GuiEventListener.KEY_F21,
    Qt.Key_F22: agxSDK.GuiEventListener.KEY_F22,
    Qt.Key_F23: agxSDK.GuiEventListener.KEY_F23,
    Qt.Key_F24: agxSDK.GuiEventListener.KEY_F24,
    Qt.Key_F25: agxSDK.GuiEventListener.KEY_F25,
    Qt.Key_F26: agxSDK.GuiEventListener.KEY_F26,
    Qt.Key_F27: agxSDK.GuiEventListener.KEY_F27,
    Qt.Key_F28: agxSDK.GuiEventListener.KEY_F28,
    Qt.Key_F29: agxSDK.GuiEventListener.KEY_F29,
    Qt.Key_F30: agxSDK.GuiEventListener.KEY_F30,
    Qt.Key_F31: agxSDK.GuiEventListener.KEY_F31,
    Qt.Key_F32: agxSDK.GuiEventListener.KEY_F32,
    Qt.Key_F33: agxSDK.GuiEventListener.KEY_F33,
    Qt.Key_F34: agxSDK.GuiEventListener.KEY_F33,
    Qt.Key_F35: agxSDK.GuiEventListener.KEY_F35
}

