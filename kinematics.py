
from .common import output
from math import cos, sin, radians
import mathutils

def forword(context):
    if not "target" in context.scene.objects :
        return

    target = context.scene.objects["target"]
    link0 = context.scene.objects["link0"]

    # 将物体移动到基座
    target.matrix_world = link0.matrix_world
    vscale = link0.matrix_world.to_scale()
    invert_scale = mathutils.Matrix([
        [vscale[0],0,0,0],
        [0,vscale[1],0,0],
        [0,0,vscale[2],0],
        [0,0,0,1]
    ])
    invert_scale.invert()
    target.matrix_world*= invert_scale

    # 从基座移动到手腕
    for jointN in range(1,7)  :
        jointDHN = getattr(context.scene, "joint"+str(jointN)+"_DH")
        jointDHPre = getattr(context.scene, "joint"+str(jointN-1)+"_DH")

        aPre = jointDHPre[0]
        αPre = radians(jointDHPre[1])
        dN = jointDHN[2]
        θN = radians(jointDHN[3])

        T = mathutils.Matrix([
            [            cos(θN),            -sin(θN),            0,           aPre ],
            [ sin(θN)*cos(αPre),  cos(θN)*cos(αPre),  -sin(αPre),  -dN*sin(αPre) ],
            [ sin(θN)*sin(αPre),  cos(θN)*sin(αPre),   cos(αPre),   dN*cos(αPre) ],
            [                   0,                    0,           0,               1 ]
        ])

        target.matrix_world*= T

    # 从手腕移动到机械臂末端
    target.matrix_world*= mathutils.Matrix([
            [1,0,0,0],
            [0,1,0,0],
            [0,0,1,context.scene.joint6_DH[0]],
            [0,0,0,1]
        ])