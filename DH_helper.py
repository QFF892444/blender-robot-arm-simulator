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

    if endpoint1==None or endpoint2==None:
        return

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
    0: { "O": None, "z-unit": None, "x-unit": None, "y-unit": None, "H": None } ,
    1: { "O": None, "z-unit": None, "x-unit": None, "y-unit": None, "H": None } ,
    2: { "O": None, "z-unit": None, "x-unit": None, "y-unit": None, "H": None } ,
    3: { "O": None, "z-unit": None, "x-unit": None, "y-unit": None, "H": None } ,
    4: { "O": None, "z-unit": None, "x-unit": None, "y-unit": None, "H": None } ,
    5: { "O": None, "z-unit": None, "x-unit": None, "y-unit": None, "H": None } ,
    6: { "O": None, "z-unit": None, "x-unit": None, "y-unit": None, "H": None } ,
}



def zAxesLine(jointNumber) :
    link1 = bpy.context.scene.objects["link"+str(jointNumber)]
    return ( link1.matrix_world * Vector((0, 0, 0)), link1.matrix_world * Vector((0,0,50)) )

def calculateJointCoordinateSystem(jointN) :

    # 关节n 和 n-1 的坐标系
    cosysN = joints_cosys[jointN]

    # 关节 n 和 n-1 的 z轴线段
    zn = zAxesLine(jointN)
    zpre = zAxesLine(jointN-1)

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

        cosysN["O"] = zn[0]
        cosysN["z-unit"] = 50 * normalize(zn[1]-zn[0]) + zn[0]

        # 将 关节n的原点 到 关节n-1 z轴上的垂线 做为关节n 的x轴
        cosysN["H"] = projection(zn[0]-zpre[0], zpre[1]-zpre[0]) + zpre[0]
        cosysN["x-unit"] = 50 * normalize(zn[0] - cosysN["H"]) + zn[0]


    else :
        magnitude2 = v.magnitude * v.magnitude

        tn = (p2-p1).cross(d2).dot(v) / magnitude2
        tpre = (p2-p1).cross(d1).dot(v) / magnitude2

        # 带入射线函数，求出 r1 和 r2
        r1 = p1+ tn * d1
        r2 = p2+ tpre * d2

        # D-H 参数中的 a
        line_a = r1-r2

        cosysN["O"] = r1
        cosysN["z-unit"] = 50 * normalize(zn[1]-r1) + r1

        if line_a.magnitude<0.001 : # r1, r2 为同一个点，则两个 z轴相交
            # 用两轴的叉乘向量做为关节n的 x轴方向
            cosysN["x-unit"] = 50 * normalize( (zn[1]-zn[0]).cross( zpre[1]-zpre[0] ) ) + r1
        else:
            cosysN["x-unit"] = 50 * normalize(line_a) + r1

        cosysN["H"] = r2



    # # 计算 D-H 参数里的 a, alpha 和 d
    jointHDParam = getattr(bpy.context.scene, "joint"+str(jointN)+"_DH")

    # 参数a
    jointHDParam[0] = (cosysN["O"]-cosysN["H"]).magnitude

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
        jointHDParam[2] = (cosysNext["H"]-cosysN["H"]).magnitude


def normalize(v):
    v.normalize()
    return v

# 计算两直线的公垂线
# 算法参考 《3D数学基础》P268
# r1(t1) 为 zn上的垂足
# r2(t2) 为 zpre 上的垂足
# 如果两条直线相交，r1和r2实际为同一点，公垂线长度为0，v则提供了其方向
def commonPerpendicular(p1, d1, p2, d2) :

    v = d1.cross(d2)

    # d1 x d2 的长度为0(受float精度的影响接近0),表示前后两个关节的 z轴平行 或 重叠
    # 该情况下，不存在公垂线
    if v.magnitude<0.001 :
        return (None, None, None)

    magnitude2 = v.magnitude * v.magnitude

    tn = (p2 - p1).cross(d2).dot(v) / magnitude2
    tpre = (p2 - p1).cross(d1).dot(v) / magnitude2

    # 带入射线函数，求出 r1 和 r2
    r1 = p1 + tn * d1
    r2 = p2 + tpre * d2

    return (r1, r2, v)

# 计算两直线的夹角
def linesAngle(d1, d2) :

    acos_value = d1.dot(d2)/(d1.magnitude*d2.magnitude)
    # 由于精度问题， 容易出现 1.0000000000000003 这样的数值
    if acos_value>1 :
        acos_value = 1.0
    if acos_value<-1 :
        acos_value = -1.0
    return math.acos( acos_value ) * 180.0/math.pi

# 计算向量 v 到 n 的投影
def projection(v, n):
    return n * ((v * n) / (n.magnitude * n.magnitude))


# 返回关节 jointN 的z轴射线表达式
# 线段的射线表示法：
# r(t) = p  + td
# t = 0~1
def zAxes(jointN):
    link = bpy.context.scene.objects["link" + str(jointN)]
    p = link.matrix_world * Vector((0, 0, 0))
    d = link.matrix_world * Vector((0, 0, 50)) - p
    return (p, d)

# 测定 DH参数模型中的常量值： a, alpha, d
def measureDHConstValue(jointN):

    # 关节n 和 n+1 的坐标系
    cosysN = joints_cosys[jointN]

    # 关节n 和 关节n+1 的z轴射线表达式参数
    (pN, dN) = zAxes(jointN)
    (pNext, dNext) = zAxes(jointN+1)

    (hN, hNext, hDirection) = commonPerpendicular(pN,dN, pNext,dNext)

    # 没有共垂线，两轴平行或重叠
    if hN==None and hNext==None :

        # n关节的坐标，可以时 n+1 z轴上的任意位置
        cosysN["O"] = pNext
        cosysN["z-unit"] = 50 * normalize(dNext) + cosysN["O"]
        cosysN["H"] = projection(pNext-pN, dN) + pN
        cosysN["x-unit"] = 50 * normalize(cosysN["O"]-cosysN["H"]) + cosysN["O"]


    # 存在公垂线
    else :

        # D-H 参数中的 a
        line_a = hNext - hN

        # 按照DH模型的约定，关节n 的原点，在关节n+1的 z轴上
        cosysN["O"] = hNext
        cosysN["H"] = hN
        cosysN["z-unit"] = 50 * normalize(pNext+dNext-cosysN["O"]) + cosysN["O"]
        cosysN["x-unit"] = 50 * normalize(hDirection) + cosysN["O"]

    ## 计算 D-H 参数里的 a, alpha 和 d
    jointHDParam = getattr(bpy.context.scene, "joint"+str(jointN)+"_DH")
    cosysPre = joints_cosys[jointN - 1]

    jointHDParam[0] = (cosysN["O"]-cosysN["H"]).magnitude       # 参数 a
    jointHDParam[1] = linesAngle(dN, dNext)                     # 参数alpha
    jointHDParam[2] = (cosysN["H"]-cosysPre["O"]).magnitude     # 参数d
    jointHDParam[3] = linesAngle(cosysPre["x-unit"]-cosysPre["O"], cosysN["x-unit"]-cosysN["O"])
    #                                                           # 参数theta


def drawJointDHGuide(jointIdx):

    # z axes
    line(joints_cosys[jointIdx]["O"], joints_cosys[jointIdx]["z-unit"], "z-axes")
    # x axes
    line(joints_cosys[jointIdx]["O"], joints_cosys[jointIdx]["x-unit"], "x-axes")
    # line a
    if joints_cosys[jointIdx]["H"] != None :
        line(joints_cosys[jointIdx]["O"], joints_cosys[jointIdx]["H"], "DH-a")
    # line d
    if jointIdx<6 :
        line(joints_cosys[jointIdx]["H"], joints_cosys[jointIdx-1]["O"], "DH-d")
    else :
        line(joints_cosys[jointIdx]["O"], bpy.context.scene.objects["arm-end"].location, "DH-d")




def redrawGuide():

    clearGuide()

    objects = bpy.context.scene.objects

    joints_cosys[0]["O"] = objects["link0"].location
    joints_cosys[0]["z-unit"] = joints_cosys[0]["O"] + Vector((0,0,50))
    joints_cosys[0]["x-unit"] = joints_cosys[0]["O"] + Vector((50,0,0))

    for idx in range(1,6) :
        measureDHConstValue(idx)

    joints_cosys[6]["O"] = objects["arm-end"].location
    joints_cosys[6]["H"] = objects["arm-end"].location

    for idx in range(6,0,-1) :
        if getattr(bpy.context.scene, "joint"+str(idx)+"_drawDHGuide") == True :
            drawJointDHGuide(idx)

    # 最后一个关节的参数d 为到末端的距离
    # getattr(bpy.context.scene, "joint6_DH")[2] = (bpy.context.scene.objects["arm-end"].location - joints_cosys[6]["O"]).magnitude

