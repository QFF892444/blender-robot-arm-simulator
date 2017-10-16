from sympy import *
from .common import output, load
from math import radians
import bpy, time
import mathutils
import math, importlib





def transformMatrixWithoutθ(jointN) :

    scene = bpy.context.scene

    θN = Symbol("θ"+str(jointN))

    aPre = Symbol("a"+str(jointN-1))
    αPre = Symbol("α"+str(jointN-1))
    dN = Symbol("d"+str(jointN))
    # aPre = getattr(scene, "joint"+str(jointN-1)+"_DH") [0]
    # αPre = radians(getattr(scene, "joint"+str(jointN-1)+"_DH")[1])
    # dN = getattr(scene, "joint"+str(jointN)+"_DH") [2]

    return ( Matrix([
        [           cos(θN),          -sin(θN),          0,          aPre ],
        [ sin(θN)*cos(αPre), cos(θN)*cos(αPre), -sin(αPre), -dN*sin(αPre) ],
        [ sin(θN)*sin(αPre), cos(θN)*sin(αPre),  cos(αPre),  dN*cos(αPre) ],
        [                 0,                 0,          0,             1 ]
    ]), θN)

# 代入 α, a, d 常数，生成雅可比矩阵，
# 不代入 θ 变量
def buildJacobianMatrixWithoutθ() :

    matrixes = []

    listθ = (
        Symbol("θ1"),Symbol("θ2"), Symbol("θ3"), Symbol("θ4"), Symbol("θ5"), Symbol("θ6")
    )

    for i in range(1,7) :

        jointDHN = getattr(bpy.context.scene, "joint"+str(i)+"_DH")
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

    diffradian = 0.1
    jointNumber = 2

    # 构建相对于最后一个关节(而不是世界坐标系)的 Jacobian 矩阵
    JT = zeros(6,jointNumber)
    varθ = []
    for i in range(jointNumber, 0, -1) :
        if i==jointNumber :
            T = eye(4)
        else:
            TN, θ = transformMatrixWithoutθ(i+1)
            # TN = TN.subs(θ, jointVarθ(i)) # 代入θ值
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
    DT = JT * dθ

    dx, dy, dz, δx, δy, δz = DT.col(0)

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

    T01, θ1 = transformMatrixWithoutθ(1)
    T0N = T01.subs(θ1, jointVarθ(1)) * T

    dT = T0N * ΔT

    output("dT =", dT)

    # target.matrix_world = toBpy(T0N)

    global targetMatrixWorld
    # moveTargetAlongsJoints(jointNumber)
    # targetMatrixWorld = target.matrix_world.copy()
    targetMatrixWorld = toBpy(T0N)
    output("target before =")
    output(targetMatrixWorld)

    
    # 实际转动各个关节
    scene.objects["link1"].rotation_euler.z += diffradian
    scene.objects["link2"].rotation_euler.y += diffradian
    # scene.objects["link3"].rotation_euler.y += diffradian


    return
    #
    #
    #
    # T01, θ1 = transformMatrixWithoutθ(1)
    # T12, θ2 = transformMatrixWithoutθ(2)
    # # T23, θ3 = transformMatrixWithoutθ(3)
    #
    # # T13 = T12*T23
    # # T03 = T01*T13
    # # px, py, pz,_ = T03.col(3)
    # # n1z, o1z, a1z,_ = T03.row(2)
    # # n2z, o2z, a2z,_ = T13.row(2)
    # # n3z, o3z, a3z,_ = T23.row(2)
    #
    # T = T02 = T01*T12
    # px, py, pz,_ = T.col(3)
    # n1z, o1z, a1z,_ = T02.row(2)
    # n2z, o2z, a2z,_ = T12.row(2)
    #
    # # output("T03 =", T03.row(2))
    #
    # J = Matrix([
    #     # [diff(px, θ1), diff(px, θ2), diff(px, θ3)],
    #     # [diff(py, θ1), diff(py, θ2), diff(py, θ3)],
    #     # [diff(pz, θ1), diff(pz, θ2), diff(pz, θ3)],
    #     # [n1z,n2z,n3z],
    #     # [o1z,o2z,o3z],
    #     # [a1z,a2z,a3z],
    #     [diff(px, θ1), diff(px, θ2)],
    #     [diff(py, θ1), diff(py, θ2)],
    #     [diff(pz, θ1), diff(pz, θ2)],
    #     [n1z, n2z],
    #     [o1z, o2z],
    #     [a1z, a2z],
    # ])
    #
    # load("DH_helper").redrawGuide()
    #
    # D = J * Matrix([[diffradian],[diffradian]])
    # D = D.subs(θ1,jointVarθ(1))
    # D = D.subs(θ2,jointVarθ(2))
    # # D = D.subs(θ3,jointVarθ(3))
    # output("D =", D)
    #
    # Δ = Matrix([
    #     [    0, -D[5],  D[4], D[0]],
    #     [ D[5],     0, -D[3], D[1]],
    #     [-D[4],  D[3],     0, D[2]],
    #     [    0,     0,     0,    0]
    #     # [0,0,0,D[0]],
    #     # [0,0,0,D[1]],
    #     # [0,0,0,D[2]],
    #     # [0,0,0,0],
    # ])
    # output("Δ =", Δ)
    #
    #
    # output("target 1 =", target.matrix_world)
    #
    #
    #
    # moveTargetAlongsJoints(3)
    # global targetMatrixWorld
    # targetMatrixWorld = target.matrix_world.copy()
    #
    # T = toSympy(target.matrix_world)
    # dT = Δ * T
    # output("dT =", dT)
    #
    # global diffT
    # diffT = dT
    #
    # scene.objects["link1"].rotation_euler.z += diffradian
    # scene.objects["link2"].rotation_euler.y += diffradian
    # # scene.objects["link3"].rotation_euler.y += diffradian
    #
    #
    # load("DH_helper").redrawGuide()
    #
    # t = T.subs(θ1, jointVarθ(1))
    # t = t.subs(θ2, jointVarθ(2))
    # # t = t.subs(θ3, jointVarθ(3))
    # target.matrix_world = toBpy(t)


def testJacobian2():

    target = bpy.context.scene.objects["target"]

    load("DH_helper").redrawGuide()

    moveTargetAlongsJoints(2)
    output("target after =")
    output(target.matrix_world)


    global targetMatrixWorld
    realDiffT = target.matrix_world - targetMatrixWorld
    output(realDiffT)


    global diffT
    output("x精度：",realDiffT.row[0][3]-diffT.row(0)[3])
    output("y精度：",realDiffT.row[1][3]-diffT.row(1)[3])
    output("z精度：",realDiffT.row[2][3]-diffT.row(2)[3])

    return

    # load("DH_helper").redrawGuide()

    # jacobian = buildJacobianMatrix()
    # dθ = []
    # for dθN in bpy.context.scene.jointsDifferentialMotion :
    #     dθ.append(dθN)
    # Δ = makeDifferentialOperator(jacobian, dθ)
    #
    # T06 = toSympy(load("kinematics").T(1,6))

    # target = bpy.context.scene.objects["target"]
    # target.matrix_world = target.matrix_world * load("kinematics").T(1,6)

    return
    # dT = Δ * T06
    # output("T06 = ", T06)
    # output("dT = ", dT)
    #
    # link6 = bpy.context.scene.objects["link6"]
    # output("微分运动前 link6.matrix_world = ")
    # output(link6.matrix_world)
    #
    # beforeMatrixWorld = link6.matrix_world.copy()
    #
    # # 模拟执行微分运动
    # for n in range(6):
    #     dθ = bpy.context.scene.jointsDifferentialMotion[n]
    #     link = bpy.context.scene.objects["link"+str(n+1)]
    #     link.rotation_euler.z+= dθ
    #
    # link6 = bpy.context.scene.objects["link6"]
    # output("微分运动后 link6.matrix_world = ")
    # output(link6.matrix_world)

    output(beforeMatrixWorld-link6.matrix_world)



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