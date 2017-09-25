import bpy, math
from mathutils import Vector
from .common import output

class DHDrawer(bpy.types.Operator):
    bl_idname = "view3d.dh_helper_drawer"
    bl_label = "DH Helper Drawer"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        initGP(context)
        return {"FINISHED"}


def initGP(context):
    S = context.scene
    # Create grease pencil data if none exists
    if not S.grease_pencil:
        a = [a for a in bpy.context.screen.areas if a.type == 'VIEW_3D'][0]
        override = {
            'scene': S,
            'screen': bpy.context.screen,
            'object': bpy.context.object,
            'area': a,
            'region': a.regions[0],
            'window': bpy.context.window,
            'active_object': bpy.context.object
        }

        bpy.ops.gpencil.data_add(override)

    gp = S.grease_pencil

    # 创建画 H-D 辅助线的图层
    if not gp.layers or not gp.layers["H-D"]:
        gpl = gp.layers.new('H-D', set_active=True)
    else:
        gpl = gp.layers['H-D']

    # Reference active GP frame or create one of none exists
    if gpl.frames:
        fr = gpl.active_frame
    else:
        fr = gpl.frames.new(1)



########


# print("xxxx")
# initGP(bpy.context)