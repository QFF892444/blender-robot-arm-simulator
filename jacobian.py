from sympy import *
from .common import output, load, isPreposing
from math import radians
from mathutils import Vector
import bpy, time
import mathutils
import math, importlib





def transformMatrixWithoutθ(jointN) :

    scene = bpy.context.scene

    if isPreposing():
        θN = Symbol("θ"+str(jointN))
        dN = Symbol("d"+str(jointN))
        aPre = Symbol("a"+str(jointN-1))
        αPre = Symbol("α"+str(jointN-1))
        # aPre = getattr(scene, "joint"+str(jointN-1)+"_DH") [0]
        # αPre = radians(getattr(scene, "joint"+str(jointN-1)+"_DH")[1])
        # dN = getattr(scene, "joint"+str(jointN)+"_DH") [2]

        return ( Matrix([
            [           cos(θN),          -sin(θN),          0,          aPre ],
            [ sin(θN)*cos(αPre), cos(θN)*cos(αPre), -sin(αPre), -dN*sin(αPre) ],
            [ sin(θN)*sin(αPre), cos(θN)*sin(αPre),  cos(αPre),  dN*cos(αPre) ],
            [                 0,                 0,          0,             1 ]
        ]), θN)
    else :
        θ = Symbol("θ" + str(jointN))

        a = Symbol("a" + str(jointN))
        α = Symbol("α" + str(jointN))
        d = Symbol("d" + str(jointN))


        DHParams = getattr(scene, "joint"+str(jointN)+"_DH")
        a = DHParams[0]
        α = DHParams[1]
        d = DHParams[2]

        C = cos(θ)
        S = sin(θ)
        Cα = cos(α)
        Sα = sin(α)

        return (Matrix([
                [C, -S*Cα,  S*Sα, a*C] ,
                [S,  C*Cα, -C*Sα, a*S] ,
                [0,    Sα,    Cα,   d] ,
                [0,     0,     0,   1]
        ]), θ)

# 代入 α, a, d 常数，生成雅可比矩阵，
# 不代入 θ 变量
def buildJacobianMatrixWithoutθ() :

    matrixes = []

    listθ = (
        Symbol("θ1"),Symbol("θ2"), Symbol("θ3"), Symbol("θ4"), Symbol("θ5"), Symbol("θ6")
    )

    for i in range(1,7) :

        jointDHN = getattr(bpy.context.scene, "joint"+str(i)+"_DH")

        if isPreposing() :
            jointDHPre = getattr(bpy.context.scene, "joint"+str(i-1)+"_DH")

            aPre = formatNumber(jointDHPre[0])
            αPre = formatNumber(radians(jointDHPre[1]))
            dN = formatNumber(jointDHN[2])
            θN = listθ[i-1]

            T = Matrix([
                [            cos(θN),            -sin(θN),            0,           aPre ],
                [ sin(θN)*cos(αPre),  cos(θN)*cos(αPre),  -sin(αPre),  -dN*sin(αPre) ],
                [ sin(θN)*sin(αPre),  cos(θN)*sin(αPre),   cos(αPre),   dN*cos(αPre) ],
                [                   0,                    0,           0,               1 ]
            ])

        else:

            a = formatNumber(jointDHN[0])
            α = formatNumber(radians(jointDHN[1]))
            d = formatNumber(jointDHN[2])
            θ = listθ[i-1]

            C = cos(θ)
            S = sin(θ)
            Cα = cos(α)
            Sα = sin(α)

            T = Matrix([
                [C, -S*Cα,  S*Sα, a*C] ,
                [S,  C*Cα, -C*Sα, a*S] ,
                [0,    Sα,    Cα,   d] ,
                [0,     0,     0,   1]
            ])

        matrixes.append(T)

    T06 = matrixes[0]*matrixes[1]*matrixes[2]*matrixes[3]*matrixes[4]*matrixes[5]

    px = T06.col(3)[0]
    py = T06.col(3)[1]
    pz = T06.col(3)[2]


    def makeMatrixRow(equation) :
        equation = trimExpressionAdd(equation)
        row = []
        for i in range(0,6) :
            θN = listθ[i]
            partial = diff(equation, θN)
            row.append(partial)

        return row

    matrix = []
    matrix.append(makeMatrixRow(px))
    matrix.append(makeMatrixRow(py))
    matrix.append(makeMatrixRow(pz))
    matrix.append([0,0,0,0,0,0])
    matrix.append([0,0,0,0,0,0])
    matrix.append([0,0,0,0,0,0])

    jacobian = Matrix(matrix)

    return (jacobian, listθ)


jacobianMatrix = None
jacobianVarsθ = None


def buildJacobianMatrix() :
    global jacobianMatrix, jacobianVarsθ

    if jacobianMatrix==None:
        jacobianMatrix, jacobianVarsθ = buildJacobianMatrixWithoutθ()

    # 微分运动
    start = time.time()
    jacobian = jacobianMatrix.subs(jacobianVarsθ[0], jointVarθ(0))
    for i in range(1,6):
        jacobian = jacobian.subs( jacobianVarsθ[i], jointVarθ(i) )

    return jacobian


def jointVarθ(jointIdx) :
    jointDH = getattr(bpy.context.scene, "joint" + str(jointIdx) + "_DH")
    return radians(jointDH[3])

# 微分算子
def makeDifferentialOperator(jacobian, dθ) :
    D = jacobian * Matrix(dθ)
    return Matrix([
        [0,0,0,D[0]],
        [0,0,0,D[1]],
        [0,0,0,D[2]],
        [0,0,0,0],
    ])


#
def formatNumber(num) :
    intpart = round(num)
    if abs(num-intpart) > 0.001 :
        return num
    return num


def trimExpressionMul(expression) :
    simpleExpress = []
    for item in expression.args :
        if item.__class__ == Float and abs(item) < 0.00001 :
            return 0

        if item.__class__ == Add :
            item = trimExpressionAdd(item)

        simpleExpress.append(item)

    return Mul(*simpleExpress)


# 取消加法运算中微小的部分
def trimExpressionAdd(expression) :

    simpleExpress = []
    for item in expression.args :

        if item.__class__==Mul :
            item = trimExpressionMul(item)

        if item.__class__ == Float and abs(item) < 0.00001 :
            continue
        if item.__class__ == int and item==0 :
            continue

        simpleExpress.append(item)

    return trigsimp(Add(*simpleExpress))


# 生成并输出雅可比矩阵
def outputJacobianMatrix() :
    output(buildJacobianMatrix())

# 用 雅可比矩阵 计算并输出微分算子
def outputDifferentialOperator():
    jacobian = buildJacobianMatrix()
    dθ = []
    for dθN in bpy.context.scene.jointsDifferentialMotion :
        dθ.append(dθN)
    Δ = makeDifferentialOperator(jacobian, dθ)

    output("Δ = ", Δ)


diffradian = 0.1
jointNumber = 2


def moveTargetAlongsJoints(toJoints) :

    target = bpy.context.scene.objects["target"]
    target.matrix_world = mathutils.Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])
    load("kinematics").fkTransformation(1,toJoints)

    return target.matrix_world

def testJacobian():

    target = bpy.context.scene.objects["target"]
    scene = bpy.context.scene

    load("DH_helper").redrawGuide()

    output()
    output()

    # 构建相对于最后一个关节(而不是世界坐标系)的 Jacobian 矩阵
    JT = zeros(6,jointNumber)
    varθ = []
    T = eye(4)
    for i in range(jointNumber, 0, -1) :
        TN, θ = transformMatrixWithoutθ(i)
        TN = TN.subs(θ, jointVarθ(i)) # 代入θ值
        T = TN * T

        nx, ox, ax, px = T.row(0)
        ny, oy, ay, py = T.row(1)
        nz, oz, az, pz = T.row(2)

        JT[0*jointNumber+i-1] = -nx*py + ny*px
        JT[1*jointNumber+i-1] = -ox*py + oy*px
        JT[2*jointNumber+i-1] = -ax*py + ay*px
        JT[3*jointNumber+i-1] = nz
        JT[4*jointNumber+i-1] = oz
        JT[5*jointNumber+i-1] = az

    output("JT =", JT)

    dθ = Matrix( [[diffradian] for i in range(jointNumber)] )
    output("dθ=",dθ)

    DT = JT * dθ

    dx, dy, dz, δx, δy, δz = DT.col(0)
    output("dx, dy, dz, δx, δy, δz = ",dx, dy, dz, δx, δy, δz)

    ΔT = Matrix([
        [  0, -δz,  δy, dx],
        [ δz,   0, -δx, dy],
        [-δy,  δx,   0, dz],
        [  0,   0,   0,  0]
        # [0,0,0,dx],
        # [0,0,0,dy],
        # [0,0,0,dz],
        # [0,0,0,0],
    ])
    output("ΔT =",ΔT)

    dT = T * ΔT
    output("dT =", dT)

    # target.matrix_world = toBpy(T0N)

    global targetMatrixWorld
    moveTargetAlongsJoints(jointNumber)
    targetMatrixWorld = target.matrix_world.copy()
    # targetMatrixWorld = toBpy(T)
    output("target before =")
    output(targetMatrixWorld)


    # 实际转动各个关节
    for i in range(1,jointNumber+1) :
        scene.objects["link"+str(i)].rotation_euler.z += diffradian



def testJacobian2():

    target = bpy.context.scene.objects["target"]

    load("DH_helper").redrawGuide()

    moveTargetAlongsJoints(2)
    output("target after =")
    output(target.matrix_world)


    global targetMatrixWorld

    output("移动：",target.matrix_world.to_translation()-targetMatrixWorld.to_translation())
    output(targetMatrixWorld.to_euler())
    euler = eulerFromMatrix(target.matrix_world) - eulerFromMatrix(targetMatrixWorld)
    output("转动：", euler, "x="+str(euler[0]/math.pi*180), "y="+str(euler[1]/math.pi*180), "z="+str(euler[2]/math.pi*180))


    # global diffT
    # output("x精度：",realDiffT.row[0][3]-diffT.row(0)[3])
    # output("y精度：",realDiffT.row[1][3]-diffT.row(1)[3])
    # output("z精度：",realDiffT.row[2][3]-diffT.row(2)[3])



def eulerFromMatrix(m) :
    euler = m.to_euler()
    return Vector((euler.x, euler.y, euler.z))

# 将 bpy 的 Matrix 对象转换为 sympy 的 Matrxi 对象
def toSympy(m) :
    sm = []
    for row in m.row :
        srow = []
        for item in row :
            srow.append(item)
        sm.append(srow)
    return Matrix(sm)

# 将 sympy 的 Matrix 对象转换为 bpy 的 Matrxi 对象
def toBpy(m) :
    bm = []
    for ir in range(m.rows) :
        row = m.row(ir)
        srow = []
        for ic in range(row.cols) :
            srow.append(row[ic])
        bm.append(srow)
    return mathutils.Matrix(bm)