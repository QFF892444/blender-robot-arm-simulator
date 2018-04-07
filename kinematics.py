
import bpy
from .common import output, isPreposing
from math import cos, sin, radians, sqrt
from mathutils import Matrix, Euler




def fkMatrixes(context=None) :

    if context==None :
        context = bpy.context

    matrixes = []

    # 从基座移动到手腕 (关节1-6的变换矩阵)
    for jointN in range(0,7)  :

        jointDHN = getattr(context.scene, "joint"+str(jointN)+"_DH")

        if isPreposing() :

            jointDHPre = getattr(context.scene, "joint"+str(jointN-1)+"_DH")

            aPre = jointDHPre[2]
            αPre = radians(jointDHPre[3])
            dN = jointDHN[1]
            θN = radians(jointDHN[0])

            matrixes.append(Matrix([
                [           cos(θN),           -sin(θN),           0,           aPre ],
                [ sin(θN)*cos(αPre),  cos(θN)*cos(αPre),  -sin(αPre),  -dN*sin(αPre) ],
                [ sin(θN)*sin(αPre),  cos(θN)*sin(αPre),   cos(αPre),   dN*cos(αPre) ],
                [                 0,                  0,           0,              1 ]
            ]))

        else :
            matrixes.append(fkJointMatrixe(jointN, context))

    return matrixes

def fkJointMatrixe(jointN, context=None, Theta=None) :

    if context==None:
        context = bpy.context

    jointDHN = getattr(context.scene, "joint"+str(jointN)+"_DH")

    if Theta==None:
        θ = radians(jointDHN[0])
    else:
        θ = Theta
    d = jointDHN[1]
    a = jointDHN[2]
    α = radians(jointDHN[3])

    C = cos(θ)
    S = sin(θ)
    Cα = cos(α)
    Sα = sin(α)

    return Matrix([
        [C, -S*Cα,  S*Sα, a*C] ,
        [S,  C*Cα, -C*Sα, a*S] ,
        [0,    Sα,    Cα,   d] ,
        [0,     0,     0,   1]
    ])


class FakeContext : pass
def fakeContextDHModule(ThetaVars=None):

    context = FakeContext()
    setattr(context, "scene", FakeContext())
    for i in range(0,7) :
        real = getattr(bpy.context.scene, "joint"+str(i)+"_DH")
        if i>0 and ThetaVars!=None and ThetaVars[i-1]!=None :
            theta = ThetaVars[i-1]
        else:
            theta = real[0]
        joint = (theta,real[1],real[2],real[3])
        setattr(context.scene, "joint"+str(i)+"_DH", joint)

    return context



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

def forwordTarget(jointNum=None):

    if not "target" in bpy.context.scene.objects :
        return

    if jointNum==None:
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

    elif jointNum == -1 :
        bpy.context.scene.objects["target"].matrix_world = Matrix([
            [1,0,0,0] ,
            [0,1,0,0] ,
            [0,0,1,0] ,
            [0,0,0,1] ,
        ])
    else:
        bpy.context.scene.objects["target"].matrix_world = \
                bpy.context.scene.objects["target"].matrix_world * fkMatrixes() [jointNum]




def inverse(): pass


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

def T(fromJoint=None, toJoint=None, context=None) :
    if fromJoint==None:
        fromJoint = 1
    if toJoint==None:
        toJoint = 6
    t = Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])
    for i in range(fromJoint, toJoint+1) :
        TN = fkJointMatrixe(i, context)
        t*= TN
        output(i, TN)
    return t

def fkTransformation(fromJoint, toJoint) :
    target = bpy.context.scene.objects["target"]
    target.matrix_world = target.matrix_world * T(fromJoint+1, toJoint)
    return

# 各个关节的 theta 弧度值
def q():
    q = []
    for i in range(1,7):
        agree = getattr(bpy.context.scene,"joint"+str(i)+"_DH")[0]
        q.append( radians(agree) )
    return q

def forward(q) :
    T =  Matrix([
            [1,0,0,0] ,
            [0,1,0,0] ,
            [0,0,1,0] ,
            [0,0,0,1] ,
        ])
    for i in range(len(q)):
        TN = fkJointMatrixe(i+1, Theta=q[i])
        T = T * TN
    return T