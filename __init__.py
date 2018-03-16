bl_info = {
    "name": "Robot Arm Simulation",
    "category": "Object",
}


import math, importlib
import bpy
from mathutils import Vector
from bpy.props import FloatProperty, FloatVectorProperty, BoolProperty, IntProperty, StringProperty
from math import pi

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

        def pos0() :
            load("DH_helper").setJoints((0, 0, pi/2, 0,0,0))
        def posR() :
            load("DH_helper").setJoints((0, pi/2, 0, 0,-pi/2,0))
        def posS() :
            load("DH_helper").setJoints((0, 0, 0, 0,-pi/2,0))
        def posN() :
            load("DH_helper").setJoints((0, pi/4, -pi/2, 0,-pi/4,0))
        row = layout.row()
        # row.operator(InitAllJointsValue.bl_idname, text="初始姿势") \
        #     .joint_idx = -1
        func_operator(row, "初始姿势", pos0) \
                    ("就绪姿势", posR) \
                    ("伸展姿势", posS) \
                    ("灵巧姿势", posN)

        for i in range(1,7) :
            split = layout.split(percentage=0.15)
            op = split.column().operator(InitAllJointsValue.bl_idname, text="初始值") \
                            .joint_idx = i
            split.column().prop(scene, "joint" + str(i) + "_value", text="关节" + str(i) + "角度值")

        layout.separator()

        row = layout.row()
        row.prop(scene, "preposingAxesZ", text="坐标系前置")
        func_operator(row, "标定DH模型", ("DH_helper", "measureDHModel"), passContext=True)
        func_operator(row, "应用DH模型", ("DH_helper", "applyDHModel"), passContext=True)

        row = layout.row()
        row.label(" ")
        row.label("θ")
        row.label("d")
        row.label("a")
        row.label("α")
        for i in range(1,7) :
            row = layout.row()
            # row.prop(scene, "joint"+str(i)+"_drawDHGuide", text="关节"+str(i))
            row.prop(scene, "joint"+str(i)+"_DH", text="")

        layout.separator()
        row = layout.row()
        op = row.operator(ShowOrHideArm.bl_idname, text="显示机械臂")
        op.show = True
        op = row.operator(ShowOrHideArm.bl_idname, text="隐藏机械臂")
        op.show = False
        func_operator(layout.row(), "重绘 DH辅助线", ("DH_helper","redrawGuide")) \
            ("清除 DH辅助线", ("DH_helper","clearGuide"))

        def outputForwardKinematics() :
            for i, T in enumerate(load("kinematics").fkMatrixes()) :
                output(str(i-1)+"~"+str(i), load("DH_helper").formatMatrix(T))

        layout.separator()
        func_operator(layout.row(), "正运动学变换", ("kinematics","forwordTarget")) \
            ("目标归位", ("kinematics","forwordTarget"), args=[-1])
        row = layout.row()
        func_operator(row, "-1>0", ("kinematics","forwordTarget"), args=[0]) \
            ("0-1", ("kinematics","forwordTarget"), args=[1]) \
            ("6>7", ("kinematics","forwordTarget"), args=[7])

        row.prop(scene, "fkStartJoint", text="从关节")
        row.prop(scene, "fkEndJoint", text="到关节")
        func_operator(row, "正变换", lambda : load("kinematics").fkTransformation(scene.fkStartJoint,scene.fkEndJoint))

        func_operator(layout.row(), ">>>fk变换矩阵", outputForwardKinematics) \
            (">>>末端位姿(ik)", lambda : output(load("kinematics").T()) ) \
            (">>>ik求解", ("kinematics","inverse") )
        func_operator(layout.row(), ">>>DH常量", ("kinematics", "outputDHConst")) \
            (">>>target noap", lambda : output(context.scene.objects["target"].matrix_world) )

        layout.separator()
        row = layout.row()
        row.prop(scene, "jointsDifferentialMotion", text="微分运动")
        func_operator(layout.row(), ">>>雅可比矩阵", ("jacobian", "outputJacobianMatrix")) \
                        (">>>微分算子Δ", ("jacobian", "outputDifferentialOperator")) \
                        (">>>测试雅可比", ("jacobian", "testJacobian"), passContext=True) \
                        (">>>测试雅可比2", ("jacobian", "testJacobian2"), passContext=True)



# class SetJointValue(bpy.types.Operator):
#     bl_idname = "view3d.set_joint_value"
#     bl_label = "Set Joint value"
#     bl_options = {'REGISTER', 'UNDO'}
#
#     joint_index = IntProperty()
#     request_position = StringProperty(default='middle')
#
#     def execute(self, context):
#
#         if self.request_position=="min" :
#             value = - meta_joints[self.joint_index]['max']
#         elif self.request_position=="middle" :
#             value = 0
#         elif self.request_position=="max" :
#             value = meta_joints[self.joint_index]['max']
#
#         joint_name = "joint"+str(self.joint_index)+"_value"
#         frameName = "frame" + str(self.joint_index)
#
#         context.scene.objects[frameName].rotation_euler[2] = math.radians(value)
#         context.scene[joint_name] =
# value
#
#         # 更新 D-H 辅助线
#         load("DH_helper").redrawGuide()
#
#         return {"FINISHED"}


class InitAllJointsValue(bpy.types.Operator):
    bl_idname = "view3d.init_all_joints_value"
    bl_label = "Initial all Joints Value"
    bl_options = {'REGISTER', 'UNDO'}

    joint_idx = IntProperty(0)

    def execute(self, context):

        if self.joint_idx<=0 or self.joint_idx>6 :

            for idx in range(1,7) :
                joint_name = "joint"+str(idx)+"_value"
                context.scene[joint_name] = 0
                meta_joints[idx]['update'](self, context)
        else :
            joint_name = "joint"+str(self.joint_idx)+"_value"
            context.scene[joint_name] = 0
            meta_joints[self.joint_idx]['update'](self, context)

        # 更新 D-H 辅助线
        load("DH_helper").updateTheta(context)

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

def func_operator(row, text, func, passContext=False, options=None, args=[]) :

    def create_func_operator(text, func, passContext=False, options=None, args=[]):
        if isinstance(func, tuple) :
            modulename = func[0]
            funcname = func[1]
            if passContext :
                func = lambda context: getattr(load(modulename),funcname)(*([context]+args))
            else:
                func = lambda : getattr(load(modulename),funcname)(*args)
        op = row.operator(FunctionOperator.bl_idname, text=text)
        operation_funcs.append(func)
        op.funcid = len(operation_funcs)-1
        op.passContext = passContext

        if options!=None :
            for key in options :
                setattr(op, key, options[key])

        return create_func_operator

    return create_func_operator(text, func, passContext, options, args)







def createJointValueUpdate(jointIdx)  :
    def update(self, context) :
        context.scene.objects["frame"+str(jointIdx)].rotation_euler[2] = math.radians( context.scene["joint"+str(jointIdx)+"_value"] )
        # 更新 D-H
        load("DH_helper").updateTheta(context)
    return update

meta_joints = {
    1: {
        "max": 135,
        "update": createJointValueUpdate(1),
    },
    2: {
        "max": 90,
        "update": createJointValueUpdate(2),
    },
    3: {
        "max": 135,
        "update": createJointValueUpdate(3),
        "update": createJointValueUpdate(3),
    },
    4: {
        "max": 90,
        "update": createJointValueUpdate(4),
    },
    5: {
        "max": 135,
        "update": createJointValueUpdate(5),
    },
    6: {
        "max": 90,
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

    bpy.types.Scene.jointsDifferentialMotion = FloatVectorProperty(size=6)
    bpy.types.Scene.fkStartJoint = IntProperty(default=1)
    bpy.types.Scene.fkEndJoint = IntProperty(default=6)


    bpy.types.Scene.preposingAxesZ = BoolProperty(default=True)

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