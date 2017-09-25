import bpy
from .common import output



import bpy, math
from mathutils import Vector

S  = bpy.context.scene

# Create grease pencil data if none exists
if not S.grease_pencil:
    a = [ a for a in bpy.context.screen.areas if a.type == 'VIEW_3D' ][0]
    override = {
        'scene'         : S,
        'screen'        : bpy.context.screen,
        'object'        : bpy.context.object,
        'area'          : a,
        'region'        : a.regions[0],
        'window'        : bpy.context.window,
        'active_object' : bpy.context.object
    }

    bpy.ops.gpencil.data_add( override )

gp = S.grease_pencil

# 创建画 H-D 辅助线的图层
if not gp.layers or not gp.layers["H-D"]:
    gpl = gp.layers.new('H-D', set_active = True )
else:
    gpl = gp.layers['H-D']

# Reference active GP frame or create one of none exists
if gpl.frames:
    fr = gpl.active_frame
else:
    fr = gpl.frames.new(1)


def line(v1, v2) :
    # Create a new stroke
    str = fr.strokes.new()
    str.draw_mode = '3DSPACE'

    # Number of stroke points
    strokeLength = 2

    # Add points
    str.points.add(count = 2 )
    str.points[0].co = (v1)
    str.points[1].co = (v2)

    return str

###########

# 画各个关节的坐标系

link = bpy.context.scene.objects["link2"]
line( link.matrix_world*link.data.vertices[0].co, link.matrix_world*Vector((-20,0,0)) )



link = bpy.context.scene.objects["link3"]
line( link.matrix_world*link.data.vertices[0].co, link.matrix_world*link.data.vertices[1].co )