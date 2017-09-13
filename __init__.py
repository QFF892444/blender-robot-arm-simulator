bl_info = {
    "name": "Robot Arm Simulation",
    "category": "Object",
}


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

        layout.row().operator("render.render", text="初始位置")

        drawJointAngleUI(scene, layout, 1)
        drawJointAngleUI(scene, layout, 2)
        drawJointAngleUI(scene, layout, 3)
        drawJointAngleUI(scene, layout, 4)
        drawJointAngleUI(scene, layout, 5)
        drawJointAngleUI(scene, layout, 6)




def drawJointAngleUI(obj, layout, index) :
    layout.separator()
    row = layout.row()
    #row.label(text="关节"+str(index)+"角度")
    row.prop(obj, "joint"+str(index)+"_value", text="关节"+str(index)+"角度")
    row = layout.row()
    row.operator("render.render", text="最小")
    row.operator("render.render", text="中值")
    row.operator("render.render", text="最大")

def createJointValueUpdate(jointIdx)  :
    def update(self, context) :
        print("jointValueUpdate:", jointIdx, self["joint"+str(jointIdx)+"_value"])
        
    return update


def register():
    print("register()")

    bpy.types.Scene.joint1_value = FloatProperty(update=createJointValueUpdate(1))
    bpy.types.Scene.joint2_value = FloatProperty(update=createJointValueUpdate(2))
    bpy.types.Scene.joint3_value = FloatProperty(update=createJointValueUpdate(3))
    bpy.types.Scene.joint4_value = FloatProperty(update=createJointValueUpdate(4))
    bpy.types.Scene.joint5_value = FloatProperty(update=createJointValueUpdate(5))
    bpy.types.Scene.joint6_value = FloatProperty(update=createJointValueUpdate(6))

    bpy.utils.register_class(ArmControlPanel)

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