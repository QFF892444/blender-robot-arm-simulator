# import importlib
# importlib.sys.modules['blender-robot-arm-simulator.DH_helper']

import bpy, math
from mathutils import Vector
from .common import output


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

    return S.grease_pencil


def gpLayerFrame(context):

    gp = initGP(context)

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

    return fr


def gpColor(context, name):
    gp = initGP(context)

    if "joint-help-line" in gp.palettes:
        palette = gp.palettes.get("joint-help-line")
    else:
        palette = gp.palettes.new("joint-help-line")

    if name in palette.colors :
        color = palette.colors.get(name)
    else:
        color = palette.colors.new()
        color.color = Vector([0.8815780282020569, 0.20415417850017548, 0.32798585295677185])
        color.name = name

    return color


def line(endpoint1, endpoint2, colorname="default") :

    fr = gpLayerFrame(bpy.context)

    # Create a new stroke
    str = fr.strokes.new(colorname=colorname)
    str.draw_mode = '3DSPACE'

    # Add points
    str.points.add(count = 2 )
    str.points[0].co = (endpoint1)
    str.points[1].co = (endpoint2)

    return str



def clearGuide():
    fr = gpLayerFrame(bpy.context)
    if fr==None :
        return
    for str in fr.strokes.values() :
        fr.strokes.remove(str)

# 各个关节的坐标系
joints_cosys = {
    0: { "origin": None, "z-norm": None, "x-norm": None, "line-a-foot": None } ,
    1: { "origin": None, "z-norm": None, "x-norm": None, "line-a-foot": None } ,
    2: { "origin": None, "z-norm": None, "x-norm": None, "line-a-foot": None } ,
    3: { "origin": None, "z-norm": None, "x-norm": None, "line-a-foot": None } ,
    4: { "origin": None, "z-norm": None, "x-norm": None, "line-a-foot": None } ,
    5: { "origin": None, "z-norm": None, "x-norm": None, "line-a-foot": None } ,
    6: { "origin": None, "z-norm": None, "x-norm": None, "line-a-foot": None } ,
}

def zAxleLine(jointNumber) :
    link1 = bpy.context.scene.objects["link"+str(jointNumber)]
    return ( link1.matrix_world * Vector((0, 0, 0)), link1.matrix_world * Vector((0,0,50)) )

def normalize(v) :
    v.normalize()
    return v

# 计算向量 v 到 n 的投影
def projection(v,n) :
    return n * ( (v*n) /(n.magnitude * n.magnitude) )


def calculateJointCoordinateSystem(jointN) :

    # 关节n 和 n-1 的坐标系
    cosysN = joints_cosys[jointN]

    # 关节 n 和 n-1 的 z轴线段
    zn = zAxleLine(jointN)
    zpre = zAxleLine(jointN-1)

    # 线段的射线表示法：
    # r(t) = p  + td
    # t = 0~1
    # 算法参考 《3D数学基础》P268
    # r1(t1) 为 zn上的垂足
    # r2(t2) 为 zpre 上的垂足
    p1 = zn[0]
    d1 = zn[1] - zn[0]
    p2 = zpre[0]
    d2 = zpre[1] - zpre[0]

    v = d1.cross(d2)

    # d1xd2 的长度为0(受float精度的影响接近0),表示前后两个关节的 z轴平行 或 重叠
    if v.magnitude<0.001 :

        cosysN["origin"] = zn[0]
        cosysN["z-norm"] = 50 * normalize(zn[1]-zn[0]) + zn[0]

        # 将 关节n的原点 到 关节n-1 z轴上的垂线 做为关节n 的x轴
        cosysN["line-a-foot"] = projection(zn[0]-zpre[0], zpre[1]-zpre[0]) + zpre[0]
        cosysN["x-norm"] = 50 * normalize(zn[0] - cosysN["line-a-foot"]) + zn[0]


    else :
        magnitude2 = v.magnitude * v.magnitude

        tn = (p2-p1).cross(d2).dot(v) / magnitude2
        tpre = (p2-p1).cross(d1).dot(v) / magnitude2

        # 带入射线函数，求出 r1 和 r2
        r1 = p1+ tn * d1
        r2 = p2+ tpre * d2

        # D-H 参数中的 a
        line_a = r1-r2

        cosysN["origin"] = r1
        cosysN["z-norm"] = 50 * normalize(zn[1]-r1) + r1

        if line_a.magnitude<0.001 : # r1, r2 为同一个点，则两个 z轴相交
            # 用两轴的叉乘向量做为关节n的 x轴方向
            cosysN["x-norm"] = 50 * normalize( (zn[1]-zn[0]).cross( zpre[1]-zpre[0] ) ) + r1
        else:
            cosysN["x-norm"] = 50 * normalize(line_a) + r1

        cosysN["line-a-foot"] = r2



    # # 计算 D-H 参数里的 a, alpha 和 d
    jointHDParam = getattr(bpy.context.scene, "joint"+str(jointN)+"_DH")

    # 参数a
    jointHDParam[0] = (cosysN["origin"]-cosysN["line-a-foot"]).magnitude

    # 参数alpha
    vzn = zn[1] - zn[0]
    vzpre = zpre[1] - zpre[0]
    acos_value = vzn.dot(vzpre)/(vzn.magnitude*vzpre.magnitude)
    if acos_value>1 :   # 由于精度问题， 容易出现 1.0000000000000003 这样的数值
        acos_value = 1.0
    jointHDParam[1] = math.acos( acos_value ) * 180.0/math.pi

    # 1-5关节的参数d
    if jointN<6 :
        cosysNext = joints_cosys[jointN+1]
        jointHDParam[2] = (cosysNext["line-a-foot"]-cosysN["line-a-foot"]).magnitude



def drawJointDHGuide(jointIdx):

    # z axle
    line(joints_cosys[jointIdx]["origin"], joints_cosys[jointIdx]["z-norm"], "z-axle")
    # x axle
    line(joints_cosys[jointIdx]["origin"], joints_cosys[jointIdx]["x-norm"], "x-axle")
    # a line
    if joints_cosys[jointIdx]["line-a-foot"] != None :
        line(joints_cosys[jointIdx]["origin"], joints_cosys[jointIdx]["line-a-foot"], "line-a")




def redrawGuide():

    clearGuide()

    objects = bpy.context.scene.objects

    joints_cosys[0]["origin"] = objects["link0"].location
    joints_cosys[0]["z-norm"] = joints_cosys[0]["origin"] + Vector((0,0,50))
    joints_cosys[0]["x-norm"] = joints_cosys[0]["origin"] + Vector((50,0,0))

    for idx in range(6,0,-1) :
        calculateJointCoordinateSystem(idx)
        drawJointDHGuide(idx)

    # 最后一个关节的参数d 为到末端的距离
    getattr(bpy.context.scene, "joint6_DH")[2] = (bpy.context.scene.objects["arm-end"].location - joints_cosys[6]["origin"]).magnitude

