import glfw
from OpenGL.GL import *
import OpenGL.GL.shaders
from OpenGL.GLU import *
import numpy as np
import math
import numpy.linalg as lin

position = list()

class StackJoint:
    def __init__(self, name, parent):
        self.name = name
        self.offset = list()
        self.channelOrder = list()
        self.channelNum = 0
        self.parent = parent
        self.channel = list()
        self.child = list()
        self.position = list()
        self.rotMatrix = list()

    def addOffset(self, outOffset):
        self.offset.append(outOffset)

    def addChannelOrder(self, outChannel):
        self.channelOrder.append(outChannel)

    def addChannelNum(self, outChannelNum):
        self.channelNum = outChannelNum
    def getChannelNum(self):
        return self.channelNum

    def addPosition(self, otherPosition):
        self.position.append(otherPosition)

    def getPosition(self):
        return self.position

    def addRotMatrix(self, otherRotMatrix):
        self.rotMatrix.append(otherRotMatrix)
    def getRotMatrix(self):
        return self.rotMatrix

    def getOffset(self):
        return self.offset

    def addChannel(self, someList):
        self.channel.append(someList)

    def getChannelOrder(self):
        return self.channelOrder

    def getChild(self):
        return self.child

    def getChannel(self):
        return self.channel

    def getParent(self):
        return self.parent

    def addChild(self, otherChild):
        self.child.append(otherChild)

    def getName(self):
        return self.name


leftPressed = False
rightPressed = False
originX = 0
originY = 0
rotateX = 90
rotateY = 15
originPanningX = 0
originPanningY = 0
panningX = 0
panningY = 0
radius = 300
offset = np.identity(4)
mode = 1
forced = 1
vlist = list()
vnlist = list()
flist = list()
rotateMatrixf = list()
frame = 1
animate = 1
gVertexArray = list()
gNormalArray = list()
gForcedNormalArray = list()
gVertexArray = np.array(gVertexArray)
gNormalArray = np.array(gNormalArray)

jointStack = list()
pressTime = 0
# When start, it is the first value of Camera point :: I feel shame to hardcode this.
oldPointOffset = [4.73167189e-16, 2.07055236e+00, 7.72740661e+00, 1.00000000e+00]
pointOffset = [4.73167189e-16, 2.07055236e+00, 7.72740661e+00, 1.00000000e+00]
AmbientLightValue = [0.3, 0.3, 0.3, 1.0]
DiffuseLightValue = [0.7, 0.7, 0.7, 1.0]
SpecularLightValue = [1.0, 1.0, 1.0, 1.0]
PositionLightValue = [30.0, 100.0, 30.0, 0.0]


targetX = 0
targetY = 0
targetZ = 0

x = 0
y = 0
z = 0


def mouseCallback(window, button, action, mods):
    global leftPressed
    global rightPressed
    if button == glfw.MOUSE_BUTTON_LEFT:
        if action == glfw.PRESS:
            leftPressed = True
        elif action== glfw.RELEASE:
            leftPressed = False
    if button == glfw.MOUSE_BUTTON_RIGHT:
        if action == glfw.PRESS:
            rightPressed = True
        elif action== glfw.RELEASE:
            rightPressed = False

def xRotate(theta):
    theta = np.radians(-theta)
    return np.array([[1.0, 0.0, 0.0, 0.0],
                     [0.0, np.cos(theta), -np.sin(theta), 0.0],
                     [0.0, np.sin(theta), np.cos(theta), 0.0],
                     [0.0, 0.0, 0.0, 1.0]])

def yRotate(theta):
    theta = np.radians(-theta)
    return np.array([[np.cos(theta), 0.0, np.sin(theta), 0.0],
                     [0.0, 1.0, 0.0, 0.0],
                     [-np.sin(theta), 0.0, np.cos(theta), 0.0],
                     [0.0, 0.0, 0.0, 1.0]])

def zRotate(theta):
    theta = np.radians(-theta)
    return np.array([[np.cos(theta), -np.sin(theta), 0.0, 0.0],
                     [np.sin(theta), np.cos(theta), 0.0, 0.0],
                     [0.0, 0.0, 1.0, 0.0],
                     [0.0, 0.0, 0.0, 1.0]])

def dropCallback(window, paths):

    global number
    global animate

    global joinStack
    global position
    global frame
    global frametime
    global rotateMatrixf

    file = open(paths[0],"r", encoding='UTF8')

    animate = 1
    number = 0
    jointNameList =list()
    jointStack.clear()
    rotateMatrixf.clear()
    name = " "
    index = 0
    joint =None
    mode = 0
    frame = 0
    frametime = .0
    endsite = 0
    parent = None
    position.clear()
    for line in file:
        line = line.replace("\n", "")
        line = line.strip()
        line = line.split()
        # print(line)
        if "HIERARCHY" in line:
            mode = 0
            continue
        if mode == 0:
            if "ROOT" in line:
                number += 1
                name = line[1]
                jointNameList.append(name)
                joint = StackJoint(name, None)
                jointStack.append(joint)
                continue
            if "JOINT" in line:
                number += 1
                name = line[1]
                jointNameList.append(name)
                joint = StackJoint(name, parent)
                joint.getParent().addChild(joint)
                jointStack.append(joint)
                index += 1
                continue
            if "End" in line[0] and "Site" in line[1]:
                joint = StackJoint("EndSite", parent)
                joint.getParent().addChild(joint)
                jointStack.append(joint)
                index += 1
                endsite = 1
                continue
            if "{" in line:
                if endsite !=1:
                    parent = joint
                jointStack.append(StackJoint("{",None))
                continue
            if "}" in line:
                if endsite==1:
                    endsite = 0
                else:
                    joint = joint.getParent()
                    parent = joint.getParent()
                jointStack.append(StackJoint("}",None))

                continue
            if "OFFSET" in line:
                line.remove("OFFSET")
                line = list(map(float, line))
                for i in line:
                    joint.addOffset(i)
                continue
            if "CHANNELS" in line:
                line.remove("CHANNELS")
                line = line[1:]
                channelCount = 0
                for i in line:
                    if "rotation" in i.lower():
                        joint.addChannelOrder(i)
                        channelCount += 1
                joint.addChannelNum(channelCount)
                continue

        if "MOTION" in line:
            mode = 1
            continue
        if mode == 1:
            if "Frames:" == line[0]:
                frame = int(line[1])
                continue
            if "Frame" == line[0] and "Time:" == line[1]:
                frametime = float(line[2])
                continue
            else:
                line = list(map(float, line))
                jointPointer = 3
                position.append(line[0:3])
                for joint in jointStack:
                    channum = joint.getChannelNum()
                    if channum != 0:
                        joint.addPosition(line[0:3])
                        joint.addChannel(line[jointPointer:jointPointer+channum])
                        jointPointer+= channum

    count = 0
    for joint in jointStack:

        if joint.getChannelNum() == 0:
            continue

        for i in joint.getChannel():
            rotateM = np.identity(4)

            for j in range(len(i)):
                if joint.getChannelOrder()[j].lower() == "xrotation":
                    rotateM = xRotate(i[j]) @ rotateM
                if joint.getChannelOrder()[j].lower() == "yrotation":
                    rotateM = yRotate(i[j]) @ rotateM
                if joint.getChannelOrder()[j].lower() == "zrotation":
                    rotateM = zRotate(i[j]) @ rotateM

            joint.addRotMatrix(rotateM)

    rotateMatrixf = jointStack[0].getRotMatrix()

    print("Name: " + paths[0])
    print("Number of Frame: " + str(frame))
    print("FPS: " + str(1/frametime))
    print("Number of Joint: " + str(number))
    print("Joint Name List")
    for jointName in jointNameList:
        print("   " + jointName)


def cursorCallback(window, xpos, ypos):
    global rotateX, rotateY
    global originX, originY
    global originPanningX, originPanningY
    global panningX, panningY

    if leftPressed is True:

        rotateX = rotateX+(xpos - originX)
        rotateX = rotateX % 360

        rotateY = rotateY+(ypos - originY)
        rotateY = rotateY % 360

        #to avoid "Gimbal lock"
        if rotateY % 90 == 0:
            rotateY += 0.1
    originX = xpos
    originY = ypos

    if rightPressed is True:
        panningX = panningX + ((xpos-originPanningX) * 10)
        panningY = panningY + ((ypos-originPanningY) * 10)
    originPanningX = xpos
    originPanningY = ypos

def key_callback(window, key, scancode, action, mods):
    global mode
    global forced
    global animate
    global pressTime
    global objectColor, lightColor

    if action == glfw.PRESS or action == glfw.REPEAT:
        if key == glfw.KEY_Z:
            mode *= -1
        if key == glfw.KEY_S:
            forced *= -1
        if key == glfw.KEY_SPACE:
            animate *= -1
            pressTime = glfw.get_time()

def scrollCallback(window, xoffset, yoffset):
    global radius

    radius -= (yoffset * 10)

    if radius > 10000:
        radius = 10000
    if radius < 0:
        radius = 0

def myLookAt(eye, at, up):

    global pointOffset
    global oldPointOffset
    global x, y, z
    w = (eye-at)/(np.sqrt(np.dot((eye-at), (eye-at))))
    u = (np.cross(up, w)) / (np.sqrt(np.dot(np.cross(up, w), np.cross(up, w))))
    v = np.cross(w, u)

    Ma = np.array([[u[0], v[0], w[0], eye[0]],
                   [u[1], v[1], w[1],  eye[1]],
                   [u[2], v[2], w[2],  eye[2]],
                   [0.0, 0.0, 0.0, 1.0]])

    Mview = np.array([[u[0], u[1], u[2], -u@eye],
                      [v[0], v[1], v[2], -v@eye],
                      [w[0], w[1], w[2], -w@eye],
                      [0.0, 0.0, 0.0, 1.0]])

    translateView = np.array([[1.0, 0.0, 0.0, -0.015 * panningX],
                              [0.0, 1.0, 0.0, 0.015 * panningY],
                              [0.0, 0.0, 1.0, 0.0],
                              [0.0, 0.0, 0.0, 1.0]])

    offset = Ma @ translateView
    MInv = np.linalg.inv(offset)
    offset = np.transpose(offset)
    pointOffset = offset[3]
    x = oldPointOffset[0] - pointOffset[0]
    y = oldPointOffset[1] - pointOffset[1]
    z = oldPointOffset[2] - pointOffset[2]

    oldPointOffset = pointOffset
    glMultMatrixf(MInv.T)



def drawXFrame():
    glBegin(GL_LINES)
    glColor3ub(255, 255, 255)
    glVertex3fv(np.array([-400., 0., 0.]))
    glVertex3fv(np.array([400., 0., 0.]))
    glEnd()

def drawZFrame():
    glBegin(GL_LINES)
    glColor3ub(255, 255, 255)
    glVertex3fv(np.array([0, 0., -400.]))
    glVertex3fv(np.array([0, 0., 400.]))
    glEnd()

def drawFrame():
    # draw coordinate: x in red, y in green, z in blue
    glBegin(GL_LINES)

    glColor3ub(0, 255, 0)
    glVertex3fv(np.array([0., 0., 0.]))
    glVertex3fv(np.array([40, 0., 0.]))
    glColor3ub(0, 255, 0)
    glVertex3fv(np.array([0., 0., 0.]))
    glVertex3fv(np.array([0., 40, 0.]))
    glColor3ub(0, 255, 0)
    glVertex3fv(np.array([0., 0., 0]))
    glVertex3fv(np.array([0., 0., 40]))

    glEnd()

def drawLine(offset):
    glBegin(GL_LINES)
    glVertex3f(0, 0, 0)
    glVertex3fv(offset)
    glEnd()

def drawXFrameArray():
    for k in range(11):
        glPushMatrix()
        glTranslatef(0.0, 0.0, 40 * k)
        drawXFrame()
        glPopMatrix()
    for k in range(11):
        glPushMatrix()
        glTranslatef(0.0, 0.0, 40 * (-k))
        drawXFrame()
        glPopMatrix()

def drawZFrameArray():
    for k in range(11):
        glPushMatrix()
        glTranslatef(40 * k, 0.0, 0.0)
        drawZFrame()
        glPopMatrix()
    for k in range(11):
        glPushMatrix()
        glTranslatef(40 * -k, 0.0, 0.0)
        drawZFrame()
        glPopMatrix()


def draw_sphere(radius):

    glColor(1.0, 0.7, 0.0)
    # 쿼드릭 객체 생성
    quad = gluNewQuadric()
    # 구 그리기, 인자는 순서대로 쿼드릭 객체, 반지름, 경도 분할 수, 위도 분할 수
    gluSphere(quad, radius, 10, 10)
    # 사용이 끝난 쿼드릭 객체 삭제
    gluDeleteQuadric(quad)

    glColor(1.0, 1.0, 1.0)

def draw_cone(radius, height):
    # 원뿔의 설정값
    slices = 4  # 원뿔 바닥면의 분할 수(세분화 정도)
    glColor(0.0, 1.0, 1.0)
    # 원뿔 그리기 시작
    glBegin(GL_TRIANGLE_FAN)

    # 원뿔의 꼭대기
    glVertex3f(0.0, 0.0, height)

    # 원뿔 바닥면 그리기
    for i in range(slices + 1):
        angle = 2 * math.pi * i / slices
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        # 법선 벡터 설정
        normal = (x, y, 0)
        glNormal3fv(normal)
        glVertex3f(x, y, 0.0)

    glEnd()

    # 원뿔 바닥면 그리기
    glBegin(GL_POLYGON)
    # 바닥면의 법선은 z축 방향
    glNormal3f(0.0, 0.0, -1.0)
    for i in range(slices + 1):
        angle = 2 * math.pi * i / slices
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        glVertex3f(x, y, 0.0)
    glEnd()

    glColor(1.0, 1.0, 1.0)

# 두 점 사이의 방향 벡터를 계산하는 함수
def direction_vector(start, end):
    return np.array(end) - np.array(start)

# 두 벡터 사이의 각도를 계산하는 함수
def angle_between_vectors(v1, v2):
    unit_v1 = v1 / np.linalg.norm(v1)
    unit_v2 = v2 / np.linalg.norm(v2)
    dot_product = np.dot(unit_v1, unit_v2)
    angle = np.arccos(np.clip(dot_product, -1.0, 1.0))
    return np.degrees(angle)

def draw_joint_link(level, rotation_angle, rotation_axis, link_length):
    radius = 3.0 - (0.3 * level)
    if radius < 0.3:
        radius = 0.3
    height = link_length - (radius + 3.0 - (0.3 * level + 1))

    glPushMatrix()
    glRotatef(rotation_angle, rotation_axis[0], rotation_axis[1], rotation_axis[2])
    if level > 0:
        glPushMatrix()
        glRotatef(180, 1, 0, 0)
        glTranslatef(0, 0, radius)
        glMaterialfv(GL_FRONT, GL_DIFFUSE, [0.0, 1.0, 1.0, 1.0])
        draw_cone(radius, height)
        glPopMatrix()

    draw_sphere(radius)

    glPopMatrix()


def setLighting():
    glEnable(GL_LIGHTING)
    glLightfv(GL_LIGHT0, GL_AMBIENT, AmbientLightValue)
    glLightfv(GL_LIGHT0, GL_DIFFUSE, DiffuseLightValue)
    glLightfv(GL_LIGHT0, GL_SPECULAR, SpecularLightValue)
    glLightfv(GL_LIGHT0, GL_POSITION, PositionLightValue)
    glEnable(GL_LIGHT0)
    glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE)
    glMaterialfv(GL_FRONT, GL_SPECULAR, [0.0, 1.0, 1.0, 1.0])
    glMateriali(GL_FRONT, GL_SHININESS, 10)

def renderAnimation(time):
    global rotateX, rotateY
    global radius
    global offset, oldOffset
    global x, y, z
    global targetX
    global targetY
    global targetZ
    global mode
    global jointStack
    global position
    global frame
    global rotateMatrixf

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glEnable(GL_DEPTH_TEST)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()

    gluPerspective(60, 1, 1, 10000)

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    rotateXB = np.radians(rotateX)
    rotateYB = np.radians(rotateY)

    targetX += x
    targetY += y
    targetZ += z

    if rotateY < 90:
        myLookAt(np.array([radius * np.cos(rotateXB) * np.cos(rotateYB),
                           radius * np.sin(rotateYB),
                           radius * np.cos(rotateYB) * np.sin(rotateXB)]),
                 np.array([0, 0, 0]),
                 np.array([0, 1, 0]))

    elif rotateY >= 90 and rotateY < 270:
        myLookAt(np.array([radius * np.cos(rotateXB) * np.cos(rotateYB),
                           radius * np.sin(rotateYB),
                           radius * np.cos(rotateYB) * np.sin(rotateXB)]),
                 np.array([0, 0, 0]),
                 np.array([0, -1, 0]))

    elif rotateY >= 270:
        myLookAt(np.array([radius * np.cos(rotateXB) * np.cos(rotateYB),
                           radius * np.sin(rotateYB),
                           radius * np.cos(rotateYB) * np.sin(rotateXB)]),
                 np.array([0, 0, 0]),
                 np.array([0, 1, 0]))

    if mode > 0:
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
    else:
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)

    glPushMatrix()
    setLighting()
    drawFrame()
    drawXFrameArray()
    drawZFrameArray()
    glPopMatrix()

    renderJoint = StackJoint("Temp",0)
    glPushMatrix()

    if position:
        tempPos = position[time]
        tempM = np.array([[1.0, 0.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0, 0.0],
                          [0.0, 0.0, 1.0, 0.0],
                          [tempPos[0], tempPos[1], tempPos[2], 1.0]])
        glMultMatrixf(tempM)
        glMultMatrixf(rotateMatrixf[time])

    level = -1

    for joint in jointStack:
        if "{" in joint.name:
            glPushMatrix()
            offset = renderJoint.offset
            parent = renderJoint.getParent()

            level += 1

            if parent is None:
                parentOffset = [0, 0, 0]

            else:
                parentOffset = parent.getOffset()
                translateM = np.array([[1.0, 0.0, 0.0, 0.0],
                                       [0.0, 1.0, 0.0, 0.0],
                                       [0.0, 0.0, 1.0, 0.0],
                                       [offset[0], offset[1], offset[2], 1.0]])

                rotateM = np.identity(4)

                if renderJoint.getRotMatrix():
                    rotateM = renderJoint.getRotMatrix()[time]

                invertTranslateM = lin.inv(translateM)
                invertTranslateM = invertTranslateM.T

                current_direction = [0, 0, 1]
                start_point = [0, 0, 0]
                end_point = (invertTranslateM @ np.array([0, 0, 0, 1]))[:-1]
                target_direction = direction_vector(start_point, end_point)
                rotation_axis = np.cross(current_direction, target_direction)
                rotation_angle = angle_between_vectors(current_direction, target_direction)
                draw_joint_link(level, rotation_angle, rotation_axis, np.linalg.norm(end_point))

                glMultMatrixf(translateM)
                glMultMatrixf(rotateM)
                continue

        if "}" in joint.name:
            glPopMatrix()
            level -= 1
            continue
        else:
            renderJoint = joint
    glPopMatrix()

def renderCallibration():
    global rotateX, rotateY
    global radius
    global offset, oldOffset
    global x, y, z
    global targetX
    global targetY
    global targetZ
    global mode
    global jointStack
    global position
    global frame
    global rotateMatrixf
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glEnable(GL_DEPTH_TEST)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()

    gluPerspective(60, 1, 1, 10000)

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    rotateXB = np.radians(rotateX)
    rotateYB = np.radians(rotateY)

    targetX += x
    targetY += y
    targetZ += z

    if rotateY < 90:
        myLookAt(np.array([radius * np.cos(rotateXB) * np.cos(rotateYB),
                           radius * np.sin(rotateYB),
                           radius * np.cos(rotateYB) * np.sin(rotateXB)]),
                 np.array([0, 0, 0]),
                 np.array([0, 1, 0]))

    elif rotateY >= 90 and rotateY < 270:
        myLookAt(np.array([radius * np.cos(rotateXB) * np.cos(rotateYB),
                           radius * np.sin(rotateYB),
                           radius * np.cos(rotateYB) * np.sin(rotateXB)]),
                 np.array([0, 0, 0]),
                 np.array([0, -1, 0]))

    elif rotateY >= 270:
        myLookAt(np.array([radius * np.cos(rotateXB) * np.cos(rotateYB),
                           radius * np.sin(rotateYB),
                           radius * np.cos(rotateYB) * np.sin(rotateXB)]),
                 np.array([0, 0, 0]),
                 np.array([0, 1, 0]))

    if mode > 0:
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
    else:
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)

    glPushMatrix()
    setLighting()

    drawFrame()
    drawXFrameArray()
    drawZFrameArray()
    glPopMatrix()

    renderJoint = StackJoint("Temp", 0)

    glPushMatrix()
    level = -1

    for joint in jointStack:
        if "{" in joint.name:
            glPushMatrix()
            offset = renderJoint.offset
            parent = renderJoint.getParent()

            level += 1

            if parent == None:
                parentOffset = [0, 0, 0]
            else:
                parentOffset = parent.getOffset()

                translateM = np.array([[1.0, 0.0, 0.0, 0.0],
                                       [0.0, 1.0, 0.0, 0.0],
                                       [0.0, 0.0, 1.0, 0.0],
                                       [offset[0], offset[1], offset[2], 1.0]])

                invertTranslateM = lin.inv(translateM)
                invertTranslateM = invertTranslateM.T

                current_direction = [0, 0, 1]
                start_point = [0, 0, 0]
                end_point = (invertTranslateM @ np.array([0, 0, 0, 1]))[:-1]
                target_direction = direction_vector(start_point, end_point)
                rotation_axis = np.cross(current_direction, target_direction)
                rotation_angle = angle_between_vectors(current_direction, target_direction)
                draw_joint_link(level, rotation_angle, rotation_axis, np.linalg.norm(end_point))

                glMultMatrixf(translateM)
                continue

        if "}" in joint.name:
            glPopMatrix()
            level -= 1
            continue
        else:
            renderJoint = joint
    glPopMatrix()

def renderWrapper():
    global animate, frame
    global pressTime
    global frametime

    if animate > 0:
        renderCallibration()
    else:
        time = glfw.get_time()-pressTime
        renderAnimation(int(time * (1 / frametime)) % frame)
def main():
    global frame
    global animate

    if not glfw.init():
        return
    window = glfw.create_window(800, 800, "BVH_Player v2.0", None, None)
    if not window:
        glfw.terminate()
        return

    glfw.set_cursor_pos_callback(window,cursorCallback)
    glfw.set_key_callback(window,key_callback)
    glfw.set_mouse_button_callback(window,mouseCallback)
    glfw.set_input_mode(window, glfw.STICKY_MOUSE_BUTTONS, 1)
    glfw.set_scroll_callback(window,scrollCallback)
    glfw.set_drop_callback(window,dropCallback)
    glfw.make_context_current(window)
    glfw.swap_interval(1)

    while not glfw.window_should_close(window):
        glfw.poll_events()
        renderWrapper()
        glfw.swap_buffers(window)


    glfw.terminate()

if __name__ == "__main__":
    main()