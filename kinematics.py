
import bpy
from .common import output
from math import cos, sin, radians, sqrt
from mathutils import Matrix, Euler


def fkMatrixes() :

    matrixes = []

    # -1 到 0 的变换矩阵（世界坐标系原点 到 机械臂基座）
    link0 = bpy.context.scene.objects["link0"]
    matrixes.append( noscale(link0.matrix_world) )

    # 从基座移动到手腕 (关节1-6的变换矩阵)
    for jointN in range(1,7)  :
        jointDHN = getattr(bpy.context.scene, "joint"+str(jointN)+"_DH")
        jointDHPre = getattr(bpy.context.scene, "joint"+str(jointN-1)+"_DH")

        aPre = jointDHPre[0]
        αPre = radians(jointDHPre[1])
        dN = jointDHN[2]
        θN = radians(jointDHN[3])

        matrixes.append(Matrix([
            [            cos(θN),            -sin(θN),            0,           aPre ],
            [ sin(θN)*cos(αPre),  cos(θN)*cos(αPre),  -sin(αPre),  -dN*sin(αPre) ],
            [ sin(θN)*sin(αPre),  cos(θN)*sin(αPre),   cos(αPre),   dN*cos(αPre) ],
            [                   0,                    0,           0,               1 ]
        ]))

    # 从手腕移动到机械臂末端
    matrixes.append(Matrix([
        [1,0,0,0],
        [0,1,0,0],
        [0,0,1,bpy.context.scene.joint6_DH[0]],
        [0,0,0,1]
    ]))

    return matrixes


def noscale(matrix_world):
    # 构建一个缩放矩阵
    vscale = matrix_world.to_scale()
    invert_scale = Matrix([
        [vscale[0], 0, 0, 0],
        [0, vscale[1], 0, 0],
        [0, 0, vscale[2], 0],
        [0, 0, 0, 1]
    ])
    invert_scale.invert()
    return matrix_world * invert_scale

def forword():
    if not "target" in bpy.context.scene.objects :
        return

    # 世界坐标系原点
    matrix_world = Matrix([
        [1,0,0,0] ,
        [0,1,0,0] ,
        [0,0,1,0] ,
        [0,0,0,1] ,
    ])

    for T in fkMatrixes() :
       matrix_world *= T

    bpy.context.scene.objects["target"].matrix_world = matrix_world

# 完整的运动学变换过程如下：
#   noap = T(-1,0) * T(0,1) * T(1,6) * T(6,7)
# 在等式两边左乘 T(0,1)的逆矩阵，右乘 T(6,7)的逆矩阵
# 这样就在右边消去了 T(0,1) 和 T(6,7)
# 仅保留 关节1-6 的变换， 符合DH逆运动学 解析解 的求解过程
#   => T(-1,0)(-1) * noap * T(6,7)(-1) = T(-1,0)(-1) * T(0,1) * T(1,6) * T(6,7) * T(6,7)(-1)
#   => T(-1,0)(-1) * noap * T(6,7)(-1) = T(1,6)
def ikTargetNOAP() :

    matrixes = fkMatrixes()
    T0 = matrixes[0]
    T0.invert()
    T7 = matrixes[7]
    T7.invert()

    return T0 * bpy.context.scene.objects["target"].matrix_world * T7


def inverse():
    ik_noap = ikTargetNOAP()

    joint3_equation((ik_noap[0][3],ik_noap[1][3],ik_noap[2][3]))

    return


def outputDHConst():
    for i in range(1,7) :
        jointDHParam = getattr(bpy.context.scene, "joint" + str(i) + "_DH")
        a = 0 if jointDHParam[0]<0.001 else jointDHParam[0]
        if abs(jointDHParam[1])<0.001 :
            α = 0
        elif abs(jointDHParam[1]-90)<0.001 :
            α = 90
        else:
            α = jointDHParam[1]
        d = 0 if jointDHParam[3]<0.001 else jointDHParam[3]

        txt = ""
        txt+= "a"+str(i) + "=" + str(a) + "\n"
        txt+= "α"+str(i) + "=" + str(α) + "\n"
        txt+= "d"+str(i) + "=" + str(d) + "\n"
        output(txt)


def joint3_equation(p) :


    r = p[0]**2 + p[1]**2 + p[2]**2
    z = p[2]

    output(r, z)


    a = 35777.0876399966*(1.19973587555608e+46*r**2 + 4.96185261609655e+66*r - 4.96185261609655e+66*z**2 + 1.36219736164375e+70*z - 9.40524995733487e+72)
    output(a)
    #
    solutions = [
        -sqrt(-(4.56303599238343e+35*r**2 - 5.90072017847243e+40*r + 6.25e+34*z**2 - 1.52765853881836e+39*z + 1.91542106835113e+45)/(4.56303599238343e+35*r**2 - 5.73248993973102e+40*r + 6.25e+34*z**2 - 1.52765853881836e+39*z + 1.80974832093971e+45) - 25000.0*(1.23908845154539e+48*r**2 + 6.20232083951006e+68*r - 6.20232083951005e+68*z**2 + 1.51600454255498e+73*z - 9.26562876069844e+76)**0.5/(4.56303599238343e+35*r**2 - 5.73248993973102e+40*r + 6.25e+34*z**2 - 1.52765853881836e+39*z + 1.80974832093971e+45)),
        sqrt(-(4.56303599238343e+35*r**2 - 5.90072017847243e+40*r + 6.25e+34*z**2 - 1.52765853881836e+39*z + 1.91542106835113e+45)/(4.56303599238343e+35*r**2 - 5.73248993973102e+40*r + 6.25e+34*z**2 - 1.52765853881836e+39*z + 1.80974832093971e+45) - 25000.0*(1.23908845154539e+48*r**2 + 6.20232083951006e+68*r - 6.20232083951005e+68*z**2 + 1.51600454255498e+73*z - 9.26562876069844e+76)**0.5/(4.56303599238343e+35*r**2 - 5.73248993973102e+40*r + 6.25e+34*z**2 - 1.52765853881836e+39*z + 1.80974832093971e+45)),
        -sqrt(-(4.56303599238343e+35*r**2 - 5.90072017847243e+40*r + 6.25e+34*z**2 - 1.52765853881836e+39*z + 1.91542106835113e+45)/(4.56303599238343e+35*r**2 - 5.73248993973102e+40*r + 6.25e+34*z**2 - 1.52765853881836e+39*z + 1.80974832093971e+45) + 25000.0*(1.23908845154539e+48*r**2 + 6.20232083951006e+68*r - 6.20232083951005e+68*z**2 + 1.51600454255498e+73*z - 9.26562876069844e+76)**0.5/(4.56303599238343e+35*r**2 - 5.73248993973102e+40*r + 6.25e+34*z**2 - 1.52765853881836e+39*z + 1.80974832093971e+45)),
        sqrt(-(4.56303599238343e+35*r**2 - 5.90072017847243e+40*r + 6.25e+34*z**2 - 1.52765853881836e+39*z + 1.91542106835113e+45)/(4.56303599238343e+35*r**2 - 5.73248993973102e+40*r + 6.25e+34*z**2 - 1.52765853881836e+39*z + 1.80974832093971e+45) + 25000.0*(1.23908845154539e+48*r**2 + 6.20232083951006e+68*r - 6.20232083951005e+68*z**2 + 1.51600454255498e+73*z - 9.26562876069844e+76)**0.5/(4.56303599238343e+35*r**2 - 5.73248993973102e+40*r + 6.25e+34*z**2 - 1.52765853881836e+39*z + 1.80974832093971e+45))]

    # output(solutions)

    return