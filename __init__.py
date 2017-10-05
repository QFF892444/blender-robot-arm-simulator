bl_info = {
    "name": "Robot Arm Simulation",
    "category": "Object",
}


import math, importlib
import bpy
from mathutils import Vector
from bpy.props import FloatProperty, FloatVectorProperty, BoolProperty, IntProperty, StringProperty

from . import common
importlib.reload(common)
common.developing = True
from .common import output, auto_register, auto_unregister, load

class ArmControlPanel(bpy.types.Panel):
    """Creates a Panel in the scene context of the properties editor"""
    bl_label = "Robot Arm Control"
    bl_idname = "robot_arm_control"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_category = "Robot Arm"

    def draw(self, context):
        layout = self.layout

        scene = context.scene

        layout.row().operator("view3d.init_all_joints_value", text="初始位置")

        for i in range(1,7) :
            drawJointAngleUI(scene, layout, i)

        row = layout.row()
        row.label(" ")
        row.label("a")
        row.label("α")
        row.label("d")
        row.label("θ")
        for i in range(1,7) :
            row = layout.row()
            row.prop(scene, "joint"+str(i)+"_drawDHGuide", text=str(i-1)+"-"+str(i)+" 辅助线")
            row.prop(scene, "joint" + str(i) + "_DH", text="")

        layout.separator()
        row = layout.row()
        op = row.operator(ShowOrHideArm.bl_idname, text="显示机械臂")
        op.show = True
        op = row.operator(ShowOrHideArm.bl_idname, text="隐藏机械臂")
        op.show = False
        func_operator(layout.row(), "重绘 DH辅助线", ("DH_helper","redrawGuide")) \
            ("清除 DH辅助线", ("DH_helper","clearGuide"))

        layout.separator()
        func_operator(layout.row(), "正运动学", ("kinematics","forword"), passContext=True)
        func_operator(layout.row(), ">>>DH常量", ("DH_helper", "outputDHConst")) \
            (">>>DH变换矩阵", ("DH_helper", "outputDHEquation")) \
            (">>>目标noa", lambda :
                output(load("DH_helper").formatMatrix(context.scene.objects["target"].matrix_world)) \
                          if "target" in context.scene.objects \
                          else output("missing object named 'target'")
            )

        layout.separator()
        func_operator(layout.row(), "执行脚本", lambda : load("DH_helper"))


def drawJointAngleUI(obj, layout, index) :
    layout.separator()
    row = layout.row()
    row.prop(obj, "joint"+str(index)+"_value", text="关节"+str(index)+"角度值")

    row = layout.row()
    op = row.operator("view3d.set_joint_value", text="最小")
    op.request_position = "min"
    op.joint_index = index
    op = row.operator("view3d.set_joint_value", text="中值")
    op.request_position = "middle"
    op.joint_index = index
    op = row.operator("view3d.set_joint_value", text="最大")
    op.request_position = "max"
    op.joint_index = index


def createJointValueUpdate(jointIdx)  :
    def update(self, context) :
        context.scene.objects["link"+str(jointIdx)].rotation_euler[meta_joints[jointIdx]["axle"]] = math.radians( context.scene["joint"+str(jointIdx)+"_value"] )
        # 更新 D-H 辅助线
        load("DH_helper").redrawGuide()
    return update




class SetJointValue(bpy.types.Operator):
    bl_idname = "view3d.set_joint_value"
    bl_label = "Set Joint value"
    bl_options = {'REGISTER', 'UNDO'}

    joint_index = IntProperty()
    request_position = StringProperty(default='middle')

    def execute(self, context):

        if self.request_position=="min" :
            value = - meta_joints[self.joint_index]['max']
        elif self.request_position=="middle" :
            value = 0
        elif self.request_position=="max" :
            value = meta_joints[self.joint_index]['max']

        joint_name = "joint"+str(self.joint_index)+"_value"
        linkName = "link" + str(self.joint_index)

        axle = meta_joints[self.joint_index]["axle"]
        context.scene.objects[linkName].rotation_euler[axle] = math.radians(value)
        context.scene[joint_name] = value

        # 更新 D-H 辅助线
        load("DH_helper").redrawGuide()

        # 更新 Theta 值
        jointDHParam = getattr(bpy.context.scene, "joint" + str(self.joint_index) + "_DH")

        return {"FINISHED"}


class InitAllJointsValue(bpy.types.Operator):
    bl_idname = "view3d.init_all_joints_value"
    bl_label = "Initial all Joints Value"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):

        for idx in range(1,7) :
            joint_name = "joint"+str(idx)+"_value"
            context.scene[joint_name] = 0
            meta_joints[idx]['update'](self, context)

            # 更新 Theta 值
            jointDHParam = getattr(bpy.context.scene, "joint" + str(idx) + "_DH")

        # 更新 D-H 辅助线
        load("DH_helper").redrawGuide()

        return {"FINISHED"}


class ShowOrHideArm(bpy.types.Operator):
    bl_idname = "view3d.show_or_hide_arm"
    bl_label = "Show or Hide Arm"
    bl_options = {'REGISTER', 'UNDO'}
    show = BoolProperty(default=False)

    def execute(self, context):
        for i in list(range(5)) + list(range(10,15)) :
            context.scene.layers[i] = self.show
        return {"FINISHED"}


operation_funcs = []
class FunctionOperator(bpy.types.Operator):
    bl_idname = "view3d.function_operator"
    bl_label = "some works"
    bl_options = {'REGISTER', 'UNDO'}

    funcid = IntProperty(-1)
    passContext = BoolProperty(False)

    def execute(self, context):
        if self.funcid>=0:
            if self.passContext:
                operation_funcs[self.funcid](context)
            else:
                operation_funcs[self.funcid]()
        return {"FINISHED"}

def func_operator(row, text, func, passContext=False) :
    def create_func_operator(text, func):
        if isinstance(func, tuple) :
            modulename = func[0]
            funcname = func[1]
            if passContext :
                func = lambda context: getattr(load(modulename),funcname)(context)
            else:
                func = lambda : getattr(load(modulename),funcname)()
        op = row.operator(FunctionOperator.bl_idname, text=text)
        operation_funcs.append(func)
        op.funcid = len(operation_funcs)-1
        op.passContext = passContext

        return create_func_operator
    return create_func_operator(text, func)






meta_joints = {
    1: {
        "max": 135,
        "axle": 2,
        "update": createJointValueUpdate(1),
    },
    2: {
        "max": 90,
        "axle": 1,
        "update": createJointValueUpdate(2),
    },
    3: {
        "max": 135,
        "axle": 1,
        "update": createJointValueUpdate(3),
    },
    4: {
        "max": 90,
        "axle": 2,
        "update": createJointValueUpdate(4),
    },
    5: {
        "max": 135,
        "axle": 1,
        "update": createJointValueUpdate(5),
    },
    6: {
        "max": 90,
        "axle": 2,
        "update": createJointValueUpdate(6),
    },

}

addon_keymaps = []

def register():

    print("register()",__name__)

    bpy.types.Scene.joint1_value = FloatProperty(update=meta_joints[1]["update"])
    bpy.types.Scene.joint2_value = FloatProperty(update=meta_joints[2]["update"])
    bpy.types.Scene.joint3_value = FloatProperty(update=meta_joints[3]["update"])
    bpy.types.Scene.joint4_value = FloatProperty(update=meta_joints[4]["update"])
    bpy.types.Scene.joint5_value = FloatProperty(update=meta_joints[5]["update"])
    bpy.types.Scene.joint6_value = FloatProperty(update=meta_joints[6]["update"])

    bpy.types.Scene.joint0_DH = FloatVectorProperty(size=4, default=(0,0,0,0)) # alpha0, a0 一般习惯设定为0
    bpy.types.Scene.joint1_DH = FloatVectorProperty(size=4)
    bpy.types.Scene.joint2_DH = FloatVectorProperty(size=4)
    bpy.types.Scene.joint3_DH = FloatVectorProperty(size=4)
    bpy.types.Scene.joint4_DH = FloatVectorProperty(size=4)
    bpy.types.Scene.joint5_DH = FloatVectorProperty(size=4)
    bpy.types.Scene.joint6_DH = FloatVectorProperty(size=4)

    # bpy.types.Scene.DH_a = FloatVectorProperty(size=6)
    # bpy.types.Scene.DH_alpha = FloatVectorProperty(size=6)
    # bpy.types.Scene.DH_d = FloatVectorProperty(size=6)
    # bpy.types.Scene.DH_theta = FloatVectorProperty(size=6)

    def DHGuideUpdate(self, context):
        load("DH_helper").redrawGuide()
    bpy.types.Scene.joint1_drawDHGuide = BoolProperty(default=True, update=DHGuideUpdate)
    bpy.types.Scene.joint2_drawDHGuide = BoolProperty(default=True, update=DHGuideUpdate)
    bpy.types.Scene.joint3_drawDHGuide = BoolProperty(default=True, update=DHGuideUpdate)
    bpy.types.Scene.joint4_drawDHGuide = BoolProperty(default=True, update=DHGuideUpdate)
    bpy.types.Scene.joint5_drawDHGuide = BoolProperty(default=True, update=DHGuideUpdate)
    bpy.types.Scene.joint6_drawDHGuide = BoolProperty(default=True, update=DHGuideUpdate)

    # 自动注册所有类
    auto_register(__name__)

    # handle the keymap
    wm = bpy.context.window_manager
    kc = wm.keyconfigs.addon
    if kc.keymaps.get("3D View") == None:
        km = kc.keymaps.new('3D View')
    else:
        km = kc.keymaps['3D View']


def unregister():

    # 自动注销所有类
    auto_unregister(__name__)

    for km, kmi in addon_keymaps:
        km.keymap_items.remove(kmi)
    addon_keymaps.clear()


if __name__ == "__main__":
    register()