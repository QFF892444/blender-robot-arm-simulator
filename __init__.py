bl_info = {
    "name": "Robot Arm Simulation",
    "category": "Object",
}


import math, sys, importlib
import bpy
from mathutils import Vector
from bpy.props import FloatProperty, FloatVectorProperty, BoolProperty, IntProperty, StringProperty
from .common import output, auto_register, auto_unregister
from . import DH_helper
import addon_utils


developing = True

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

        # op = row.prop(obj, "joint" + str(index) + "_drawDHGuide", text="辅助线")
        # row = layout.row()
        # v = row.prop(obj, "joint" + str(index) + "_DH", text="a,α,d,θ")

        layout.separator()
        row = layout.row()
        op = row.operator(ShowOrHideArm.bl_idname, text="显示机械臂")
        op.show = True
        op = row.operator(ShowOrHideArm.bl_idname, text="隐藏机械臂")
        op.show = False

        layout.separator()
        layout.row().operator(DrawGuide.bl_idname, text="重绘 D-H 辅助线")
        layout.row().operator(ClearGuide.bl_idname, text="清除 D-H 辅助线")


        layout.separator()
        row = layout.row()
        row.operator(OuputDHEquation.bl_idname, text=">>>D-H变换矩阵")
        row.operator(OuputTargetNOA.bl_idname, text=">>>目标noa")

        layout.separator()
        layout.row().operator(RunScript.bl_idname, text="执行脚本")
        layout.separator()
        layout.row().operator(TestButton1.bl_idname, text="测试1")
        layout.row().operator(TestButton2.bl_idname, text="测试2")
        layout.row().operator(TestButton3.bl_idname, text="测试3")


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
        loadHelper().redrawGuide()

        # 更新 Theta 值
        jointDHParam = getattr(bpy.context.scene, "joint" + str(jointIdx) + "_DH")

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
        loadHelper().redrawGuide()

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
        loadHelper().redrawGuide()

        return {"FINISHED"}


class ShowOrHideArm(bpy.types.Operator):
    bl_idname = "view3d.show_or_hide_arm"
    bl_label = "xxxxx"
    bl_options = {'REGISTER', 'UNDO'}
    show = BoolProperty(default=False)

    def execute(self, context):
        for i in list(range(5)) + list(range(10,15)) :
            context.scene.layers[i] = self.show
        return {"FINISHED"}


class DrawGuide(bpy.types.Operator):
    bl_idname = "view3d.robotarm_drawguide"
    bl_label = "xxxxx"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        loadHelper().redrawGuide()
        return {"FINISHED"}

class ClearGuide(bpy.types.Operator):
    bl_idname = "view3d.robotarm_clearguide"
    bl_label = "重绘DH辅助线"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        loadHelper().clearGuide()
        return {"FINISHED"}


class OuputDHEquation(bpy.types.Operator):
    bl_idname = "view3d.output_dh_equation"
    bl_label = "化减并输出DH方程式"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        loadHelper().outputDHEquation()
        return {"FINISHED"}

class OuputTargetNOA(bpy.types.Operator):
    bl_idname = "view3d.output_target_noa"
    bl_label = "output_target_noa"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        if "target" in context.scene.objects :
            output( loadHelper().formatMatrix( context.scene.objects["target"].matrix_world ) )
        else :
            output("missing object named 'target'")
        return {"FINISHED"}

scriptcache = {}
def loadHelper() :
    modulename = "DH_helper"
    if modulename in scriptcache:
        importlib.reload(scriptcache[modulename])
    else:
        from . import DH_helper
        scriptcache[modulename] = DH_helper
    return scriptcache[modulename]

class RunScript(bpy.types.Operator):
    bl_idname = "view3d.run_my_script"
    bl_label = "run my script"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        loadHelper()
        return {"FINISHED"}



class TestButton1(bpy.types.Operator):
    bl_idname = "view3d.test_btn_1"
    bl_label = "run my test function"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        output(dir(scriptcache["DH_helper"]))
        scriptcache["DH_helper"].clearAllLines()

        return {"FINISHED"}

class TestButton2(bpy.types.Operator):
    bl_idname = "view3d.test_btn_2"
    bl_label = "run my test function"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        output(dir(scriptcache["DH_helper"]))
        output( scriptcache["DH_helper"].gpColor(context) )

        return {"FINISHED"}

class TestButton3(bpy.types.Operator):
    bl_idname = "view3d.test_btn_3"
    bl_label = "run my test function"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):

        return {"FINISHED"}


meta_joints = {
    1: {
        "max": 135,
        "axle": 2,
        "update": createJointValueUpdate(1)
    },
    2: {
        "max": 90,
        "axle": 1,
        "update": createJointValueUpdate(2)
    },
    3: {
        "max": 135,
        "axle": 1,
        "update": createJointValueUpdate(3)
    },
    4: {
        "max": 90,
        "axle": 2,
        "update": createJointValueUpdate(4)
    },
    5: {
        "max": 135,
        "axle": 1,
        "update": createJointValueUpdate(5)
    },
    6: {
        "max": 90,
        "axle": 2,
        "update": createJointValueUpdate(6)
    },

}

addon_keymaps = []

classes = [
    ArmControlPanel ,
    SetJointValue ,
    InitAllJointsValue ,
    OuputTargetNOA ,
    OuputTargetNOA ,
    OuputTargetNOA ,
    OuputTargetNOA ,
]

def register():

    bpy.types.Scene.joint1_value = FloatProperty(update=meta_joints[1]["update"])
    bpy.types.Scene.joint2_value = FloatProperty(update=meta_joints[2]["update"])
    bpy.types.Scene.joint3_value = FloatProperty(update=meta_joints[3]["update"])
    bpy.types.Scene.joint4_value = FloatProperty(update=meta_joints[4]["update"])
    bpy.types.Scene.joint5_value = FloatProperty(update=meta_joints[5]["update"])
    bpy.types.Scene.joint6_value = FloatProperty(update=meta_joints[6]["update"])

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

    bpy.types.Scene.joint1_drawDHGuide = BoolProperty(default=True)
    bpy.types.Scene.joint2_drawDHGuide = BoolProperty(default=True)
    bpy.types.Scene.joint3_drawDHGuide = BoolProperty(default=True)
    bpy.types.Scene.joint4_drawDHGuide = BoolProperty(default=True)
    bpy.types.Scene.joint5_drawDHGuide = BoolProperty(default=True)
    bpy.types.Scene.joint6_drawDHGuide = BoolProperty(default=True)

    # 自动注册所有类
    auto_register(TestButton2.__module__)


    # handle the keymap
    wm = bpy.context.window_manager
    kc = wm.keyconfigs.addon
    if kc.keymaps.get("3D View") == None:
        km = kc.keymaps.new('3D View')
    else:
        km = kc.keymaps['3D View']


def unregister():

    # 自动注销所有类
    auto_unregister(TestButton2.__module__)

    for km, kmi in addon_keymaps:
        km.keymap_items.remove(kmi)
    addon_keymaps.clear()


if __name__ == "__main__":
    register()