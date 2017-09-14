bl_info = {
    "name": "Robot Arm Simulation",
    "category": "Object",
}


import math
import bpy
from mathutils import Vector
from bpy.props import FloatProperty

# store keymaps here to access after registration
addon_keymaps = []


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

        drawJointAngleUI(scene, layout, 1)
        drawJointAngleUI(scene, layout, 2)
        drawJointAngleUI(scene, layout, 3)
        drawJointAngleUI(scene, layout, 4)
        drawJointAngleUI(scene, layout, 5)
        drawJointAngleUI(scene, layout, 6)




def drawJointAngleUI(obj, layout, index) :
    layout.separator()
    row = layout.row()
    row.prop(obj, "joint"+str(index)+"_value", text="关节"+str(index)+"角度")

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
        print("jointValueUpdate:", jointIdx, context.scene["joint"+str(jointIdx)+"_value"])

    return update


class SetJointValue(bpy.types.Operator):
    bl_idname = "view3d.set_joint_value"
    bl_label = "Set Joint value"
    bl_options = {'REGISTER', 'UNDO'}

    joint_index = bpy.props.IntProperty()
    request_position = bpy.props.StringProperty(default='middle')

    def execute(self, context):

        if self.request_position=="min" :
            value = 0
        elif self.request_position=="middle" :
            value = meta_joints[self.joint_index]['max'] / 2
        elif self.request_position=="max" :
            value = meta_joints[self.joint_index]['max']

        joint_name = "joint"+str(self.joint_index)+"_value"
        linkName = "link" + str(self.joint_index)

        axle = meta_joints[self.joint_index]["axle"]
        context.scene.objects[linkName].rotation_euler[axle] = math.radians(value)
        context.scene[joint_name] = value

        return {"FINISHED"}


class InitAllJointsValue(bpy.types.Operator):
    bl_idname = "view3d.init_all_joints_value"
    bl_label = "Initial all Joints Value"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):

        for idx in range(1,7) :
            joint_name = "joint"+str(idx)+"_value"
            context.scene[joint_name] = meta_joints[idx]['max']/2
            meta_joints[idx]['update'](self, context)

        return {"FINISHED"}

meta_joints = {
    1: {
        "max": 270,
        "axle": 2,
        "update": createJointValueUpdate(1)
    },
    2: {
        "max": 180,
        "axle": 1,
        "update": createJointValueUpdate(2)
    },
    3: {
        "max": 270,
        "axle": 1,
        "update": createJointValueUpdate(3)
    },
    4: {
        "max": 180,
        "axle": 2,
        "update": createJointValueUpdate(4)
    },
    5: {
        "max": 270,
        "axle": 1,
        "update": createJointValueUpdate(5)
    },
    6: {
        "max": 180,
        "axle": 2,
        "update": createJointValueUpdate(6)
    },

}

def register():
    print("register()")

    bpy.types.Scene.joint1_value = FloatProperty(update=meta_joints[1]["update"])
    bpy.types.Scene.joint2_value = FloatProperty(update=meta_joints[2]["update"])
    bpy.types.Scene.joint3_value = FloatProperty(update=meta_joints[3]["update"])
    bpy.types.Scene.joint4_value = FloatProperty(update=meta_joints[4]["update"])
    bpy.types.Scene.joint5_value = FloatProperty(update=meta_joints[5]["update"])
    bpy.types.Scene.joint6_value = FloatProperty(update=meta_joints[6]["update"])

    bpy.utils.register_class(ArmControlPanel)
    bpy.utils.register_class(SetJointValue)
    bpy.utils.register_class(InitAllJointsValue)

    # handle the keymap
    wm = bpy.context.window_manager
    kc = wm.keyconfigs.addon
    if kc.keymaps.get("3D View") == None:
        km = kc.keymaps.new('3D View')
    else:
        km = kc.keymaps['3D View']
    # if kc:
        # addon_keymaps.append((km, km.keymap_items.new(CursorToSelected.bl_idname, 'R', 'PRESS', ctrl=True)))
        # addon_keymaps.append((km, km.keymap_items.new(SetOriginToSelected.bl_idname, 'Q', 'PRESS', alt=True)))
        # addon_keymaps.append((km, km.keymap_items.new(MoveSelectedsToActive.bl_idname, 'W', 'PRESS', ctrl=True)))
        # addon_keymaps.append((km, km.keymap_items.new(MoveObjectToCursor.bl_idname, 'W', 'PRESS', alt=True)))


def unregister():
    print("arm unregister()")
    bpy.utils.unregister_class(ArmControlPanel)

    for km, kmi in addon_keymaps:
        km.keymap_items.remove(kmi)
    addon_keymaps.clear()


if __name__ == "__main__":
    register()