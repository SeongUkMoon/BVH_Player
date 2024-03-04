from src.parser import Bvh
import numpy as np
import matplotlib.pyplot as plt


def _separate_angles(frames, joints, joints_saved_channels):

    frame_i = 0
    joints_saved_angles = {}
    get_channels = []
    for joint in joints:
        _saved_channels = joints_saved_channels[joint]

        saved_rotations = []
        for chan in _saved_channels:
            if chan.lower().find('rotation') != -1:
                saved_rotations.append(chan)
                get_channels.append(frame_i)

            frame_i += 1
        joints_saved_angles[joint] = saved_rotations

    joints_rotations = frames[:,get_channels]

    return joints_rotations, joints_saved_angles

def _separate_positions(frames, joints, joints_saved_channels):

    frame_i = 0
    joints_saved_positions = {}
    get_channels = []
    for joint in joints:
        _saved_channels = joints_saved_channels[joint]

        saved_positions = []
        for chan in _saved_channels:
            if chan.lower().find('position') != -1:
                saved_positions.append(chan)
                get_channels.append(frame_i)

            frame_i += 1
        joints_saved_positions[joint] = saved_positions


    if len(get_channels) == 3*len(joints):
        #print('all joints have saved positions')
        return frames[:,get_channels], joints_saved_positions

    #no positions saved for the joints or only some are saved.
    else:
        return np.array([]), joints_saved_positions

    pass

def ProcessBVH(filename):

    with open(filename) as f:
        mocap = Bvh(f.read())

    #get the names of the joints
    joints = mocap.get_joints_names()

    #this contains all of the frames data.
    frames = np.array(mocap.frames).astype('float32')

    #determine the structure of the skeleton and how the data was saved
    joints_offsets = {}
    joints_hierarchy = {}
    joints_saved_channels = {}
    for joint in joints:
        #get offsets. This is the length of skeleton body parts
        joints_offsets[joint] = np.array(mocap.joint_offset(joint))

        #Some bvh files save only rotation channels while others also save positions.
        #the order of rotation is important
        joints_saved_channels[joint] = mocap.joint_channels(joint)

        #determine the hierarcy of each joint.
        joint_hierarchy = []
        parent_joint = joint
        while True:
            parent_name = mocap.joint_parent(parent_joint)
            if parent_name == None:break

            joint_hierarchy.append(parent_name.name)
            parent_joint = parent_name.name

        joints_hierarchy[joint] = joint_hierarchy

    #seprate the rotation angles and the positions of joints
    joints_rotations, joints_saved_angles = _separate_angles(frames, joints, joints_saved_channels)
    joints_positions, joints_saved_positions = _separate_positions(frames, joints, joints_saved_channels)

    #root positions are always saved
    root_positions = frames[:, 0:3]

    return [joints, joints_offsets, joints_hierarchy, root_positions, joints_rotations, joints_saved_angles, joints_positions, joints_saved_positions]

#rotation matrices
def Rx(ang, in_radians = False):
    if in_radians == False:
        ang = np.radians(ang)

    Rot_Mat = np.array([
        [1, 0, 0],
        [0, np.cos(ang), -1*np.sin(ang)],
        [0, np.sin(ang),    np.cos(ang)]
    ])
    return Rot_Mat

def Ry(ang, in_radians = False):
    if in_radians == False:
        ang = np.radians(ang)

    Rot_Mat = np.array([
        [np.cos(ang), 0, np.sin(ang)],
        [0, 1, 0],
        [-1*np.sin(ang), 0, np.cos(ang)]
    ])
    return Rot_Mat

def Rz(ang, in_radians = False):
    if in_radians == False:
        ang = np.radians(ang)

    Rot_Mat = np.array([
        [np.cos(ang), -1*np.sin(ang), 0],
        [np.sin(ang), np.cos(ang), 0],
        [0, 0, 1]
    ])
    return Rot_Mat


#the rotation matrices need to be chained according to the order in the file
def _get_rotation_chain(joint_channels, joint_rotations):

    #the rotation matrices are constructed in the order given in the file
    Rot_Mat =  np.array([[1,0,0],[0,1,0],[0,0,1]])#identity matrix 3x3
    order = ''
    index = 0
    for chan in joint_channels: #if file saves xyz ordered rotations, then rotation matrix must be chained as R_x @ R_y @ R_z
        if chan[0].lower() == 'x':
            Rot_Mat = Rot_Mat @ Rx(joint_rotations[index])
            order += 'x'

        elif chan[0].lower() == 'y':
            Rot_Mat = Rot_Mat @ Ry(joint_rotations[index])
            order += 'y'

        elif chan[0].lower() == 'z':
            Rot_Mat = Rot_Mat @ Rz(joint_rotations[index])
            order += 'z'
        index += 1
    #print(order)
    return Rot_Mat


def _calculate_frame_joint_positions(root_position, joints, joints_offsets, frame_joints_rotations, joints_saved_angles, joints_hierarchy):

    local_positions = {}

    for joint in joints:

        #ignore root joint and set local coordinate to (0,0,0)
        if joint == joints[0]:
            local_positions[joint] = root_position
            continue

        connected_joints = joints_hierarchy[joint]
        connected_joints = connected_joints[::-1]
        connected_joints.append(joint) #this contains the chain of joints that finally end with the current joint that we want the coordinate of.
        Rot = np.eye(3)
        pos = root_position
        for i, con_joint in enumerate(connected_joints):
            if i == 0:
                pass
            else:
                parent_joint = connected_joints[i - 1]
                Rot = Rot @ _get_rotation_chain(joints_saved_angles[parent_joint], frame_joints_rotations[parent_joint])
            joint_pos = joints_offsets[con_joint]
            joint_pos = Rot @ joint_pos
            pos = pos + joint_pos

        local_positions[joint] = pos

    return local_positions


#Here root position is used as local coordinate origin.
def _calculate_frame_joint_positions_in_local_space(joints, joints_offsets, frame_joints_rotations, joints_saved_angles, joints_hierarchy):

    local_positions = {}

    for joint in joints:
        #ignore root joint and set local coordinate to (0,0,0)
        if joint == joints[0]:
            local_positions[joint] = [0,0,0]
            continue

        connected_joints = joints_hierarchy[joint]
        connected_joints = connected_joints[::-1]
        connected_joints.append(joint) #this contains the chain of joints that finally end with the current joint that we want the coordinate of.
        Rot = np.eye(3)
        pos = [0,0,0]
        for i, con_joint in enumerate(connected_joints):
            if i == 0:
                pass
            else:
                parent_joint = connected_joints[i - 1]
                Rot = Rot @ _get_rotation_chain(joints_saved_angles[parent_joint], frame_joints_rotations[parent_joint])
            joint_pos = joints_offsets[con_joint]
            joint_pos = Rot @ joint_pos
            pos = pos + joint_pos

        local_positions[joint] = pos

    return local_positions


paused = False
frame = 0
frame_skips = 10
dist = 10
bone_mode = 0
dot_enable = True
text_enable = False
captured_frame = []

from matplotlib.widgets import Slider, CheckButtons, RadioButtons
from mpl_toolkits.mplot3d.proj3d import proj_transform
from matplotlib.patches import FancyArrowPatch


class Arrow3D(FancyArrowPatch):

    def __init__(self, x, y, z, dx, dy, dz, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._xyz = (x, y, z)
        self._dxdydz = (dx, dy, dz)

    def draw(self, renderer):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        super().draw(renderer)

    def do_3d_projection(self, renderer=None):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)

        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))

        return np.min(zs)


def Draw_bvh(filename, joints, joints_offsets, joints_hierarchy, root_positions, joints_rotations, joints_saved_angles):

    global paused, frame, frame_skips, dist, bone_mode, dot_enable, text_enable, captured_frame

    def on_key(event):
        global paused, captured_frame
        if event.key == ' ':
            if paused:
                paused = False
            else:
                paused = True
        if event.key == 'e':
            if frame not in captured_frame:
                captured_frame.append(frame)


    # Window 설정
    plt.ion()
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    fig.patch.set_facecolor('#2B2D30')
    ax.set_facecolor('#1E1F22')
    fig.canvas.manager.set_window_title(f'BVH Viewer : {filename}')
    fig.canvas.toolbar.pack_forget()
    ax.view_init(elev=30, azim=45, roll=0)

    frame_joints_rotations = {en: [] for en in joints}
    figure_limit = None  # used to set figure axis limits
    fig.canvas.mpl_connect('key_press_event', on_key)
    frame = 0

    # PlayBar 설정
    plt.subplots_adjust(bottom=0.15)  # 슬라이더를 위한 공간 확보
    ax_playbar = plt.axes((0.15, 0.1, 0.73, 0.03))  # [left, bottom, width, height]
    playbar = Slider(ax_playbar, 'Frame', 0, len(joints_rotations), valinit=0, track_color="#1E1F22", color="darkorange")
    playbar.label.set_color('white')
    playbar.valtext.set_color('white')

    # SkipBar 설정
    ax_skipbar = plt.axes((0.15, 0.07, 0.73, 0.03))  # [left, bottom, width, height]
    skipbar = Slider(ax_skipbar, 'Frame Skip', 1, 20, valinit=10, track_color="#1E1F22", color="slategray")
    skipbar.label.set_color('white')
    skipbar.valtext.set_color('white')

    # DistanceBar 설정
    ax_distbar = plt.axes((0.15, 0.04, 0.73, 0.03))  # [left, bottom, width, height]
    distbar = Slider(ax_distbar, 'Distance', 5, 10, valinit=10, track_color="#1E1F22", color="slategray")
    distbar.label.set_color('white')
    distbar.valtext.set_color('white')

    # Bone Mode 설정
    radio_ax = plt.axes((0.05, 0.21, 0.1, 0.07), frameon=False)
    radio_ax.set_xticks([])
    radio_ax.set_yticks([])
    radio_ax.set_navigate(False)
    moderadio = RadioButtons(radio_ax,
                             ('Line', 'Arrow'),
                             label_props={'color': ('white', 'white'), 'fontsize': [12, 12]},
                             radio_props={'s': [64, 64], 'facecolor': ('darkorange', 'darkorange')})

    # Dot Enable 설정
    # Text Enable 설정

    check_ax = plt.axes((0.05, 0.15, 0.1, 0.07), frameon=False)
    check_ax.set_xticks([])
    check_ax.set_yticks([])
    check_ax.set_navigate(False)
    modecheck = CheckButtons(check_ax,
                             ('Joint', 'Label'),
                             actives=(True, False),
                             label_props={'color': ('white', 'white'), 'fontsize': [12, 12]},
                             check_props={'s': [64, 64], 'facecolor': ('darkorange', 'darkorange')})


    # Slider Update Function
    def update_playbar(val):
        global frame
        frame = int(playbar.val)
        playbar.val = int(frame)
        fig.canvas.draw_idle()

    def update_skipbar(val):
        global frame_skips
        frame_skips = int(skipbar.val)
        skipbar.val = int(frame_skips)
        fig.canvas.draw_idle()

    def update_distbar(val):
        global dist
        dist = distbar.val
        skipbar.val = dist
        fig.canvas.draw_idle()

    def update_mode(label):
        global bone_mode
        if label == 'Line':
            bone_mode = 0
        elif label == 'Arrow':
            bone_mode = 1

    def update_check(label):
        global dot_enable, text_enable
        if label == 'Joint':
            if dot_enable:
                dot_enable = False
            else:
                dot_enable = True
        elif label == 'Label':
            if text_enable:
                text_enable = False
            else:
                text_enable = True

    # Slider 값 변경 시 함수 연결
    playbar.on_changed(update_playbar)
    skipbar.on_changed(update_skipbar)
    distbar.on_changed(update_distbar)
    moderadio.on_clicked(update_mode)
    modecheck.on_clicked(update_check)

    def plot_frame(num, c):

        frame_data = joints_rotations[num]
        joint_index = 0
        for joint in joints:
            frame_joints_rotations[joint] = frame_data[joint_index:joint_index + 3]
            joint_index += 3

        joint_pos = _calculate_frame_joint_positions(root_positions[num], joints, joints_offsets,
                                                     frame_joints_rotations, joints_saved_angles, joints_hierarchy)

        for joint in joints:
            if joint == joints[0]: continue  # skip root joint
            parent_joint = joints_hierarchy[joint][0]

            ax.plot(xs=[joint_pos[parent_joint][0], joint_pos[joint][0]],
                    zs=[joint_pos[parent_joint][1], joint_pos[joint][1]],
                    ys=[joint_pos[parent_joint][2], joint_pos[joint][2]], c=c, lw=1.5, alpha=0.2, zorder=3)

    color = ['red', 'orange', 'yellow', 'lightgreen', 'green', 'skyblue', 'blue', 'purple', 'magenta']
    # Main Loop
    while True:

        if not paused:
            if frame + frame_skips > len(joints_rotations)-1:
                frame = 0
                captured_frame.clear()
            else:
                frame += frame_skips

        frame_data = joints_rotations[frame]
        playbar.set_val(frame)

        #fill in the rotations dict
        joint_index = 0
        for joint in joints:
            frame_joints_rotations[joint] = frame_data[joint_index:joint_index+3]
            joint_index += 3

        joint_pos = _calculate_frame_joint_positions(root_positions[frame], joints, joints_offsets, frame_joints_rotations, joints_saved_angles, joints_hierarchy)

        if figure_limit == None:
            lim_min = np.abs(np.min(joint_pos[list(joint_pos)[-1]]))
            lim_max = np.abs(np.max(joint_pos[list(joint_pos)[-1]]))
            lim = lim_min if lim_min > lim_max else lim_max
            figure_limit = lim

        x = np.linspace(-15 * figure_limit, 15 * figure_limit, 20)
        y = np.linspace(-15 * figure_limit, 15 * figure_limit, 20)
        x, y = np.meshgrid(x, y)
        z = np.zeros_like(x)
        ax.plot_surface(x, y, z, alpha=0.8, color="gray", zorder=1)  # alpha는 평면의 투명도를 조절합니다

        for joint in joints:
            if joint == joints[0]: continue #skip root joint
            parent_joint = joints_hierarchy[joint][0]

            if bone_mode == 0:
                ax.plot(xs=[joint_pos[parent_joint][0], joint_pos[joint][0]],
                        zs=[joint_pos[parent_joint][1], joint_pos[joint][1]],
                        ys=[joint_pos[parent_joint][2], joint_pos[joint][2]], c='aquamarine', lw=1.5, zorder=3)

            elif bone_mode == 1:
                u = joint_pos[joint][0] - joint_pos[parent_joint][0]
                v = joint_pos[joint][2] - joint_pos[parent_joint][2]
                w = joint_pos[joint][1] - joint_pos[parent_joint][1]

                vec = np.array([u, v, w])

                arrow = Arrow3D(joint_pos[parent_joint][0], joint_pos[parent_joint][2], joint_pos[parent_joint][1],
                                u, v, w,
                                arrowstyle=f"Simple,tail_width=0.5,head_width=2.5,head_length={np.linalg.norm(vec) * 0.65 * (1 / dist) * 10}",
                                mutation_scale=1, lw=1.0, color="aquamarine", zorder=2)
                ax.add_artist(arrow)

            if dot_enable:
                ax.plot(joint_pos[joint][0], joint_pos[joint][2], joint_pos[joint][1],
                        marker='o', markersize=2.0, c='tomato', zorder=4)

            if text_enable:
                if "Hand" not in joint:
                    ax.text(joint_pos[joint][0], joint_pos[joint][2], joint_pos[joint][1], joint,
                            fontsize=5 * (1 / dist) * 10,
                            bbox=dict(facecolor='yellow', alpha=0.5, edgecolor='green', boxstyle='round'))


        for j, cap in enumerate(captured_frame):
            plot_frame(cap, color[j % 9])

        ax.set_axis_off()
        ax.set_xlim(-10 * figure_limit, 10 * figure_limit)
        ax.set_ylim(-10 * figure_limit, 10 * figure_limit)
        ax.set_zlim(0, 20 * figure_limit)

        ax._dist = dist

        plt.pause(0.00001)
        ax.cla()

    pass

def print_hierarchy(hierarchy, node, depth=0):
    # 현재 노드와 깊이에 따라 들여쓰기를 출력합니다.
    print('   ' * depth + '-' * depth + node)

    # 현재 노드의 자식 노드들을 찾아 재귀적으로 출력합니다.
    for child in hierarchy:
        if len(hierarchy[child]) == 0 : continue
        if node == hierarchy[child][0]:
            print_hierarchy(hierarchy, child, depth + 1)

if __name__ == "__main__":

    path = "./data/custom/"
    filename = "Catalena_1.bvh"
    filepath = path + filename
    skeleton_data = ProcessBVH(filepath)

    joints = skeleton_data[0]
    joints_offsets = skeleton_data[1]
    joints_hierarchy = skeleton_data[2]
    root_positions = skeleton_data[3]
    joints_rotations = skeleton_data[4] #this contains the angles in degrees
    joints_saved_angles = skeleton_data[5] #this contains channel information. E.g ['Xrotation', 'Yrotation', 'Zrotation']
    joints_positions = skeleton_data[6]
    joints_saved_positions = skeleton_data[7]

    print(joints_rotations.shape)

    print("BVH File : " + filename)
    print(f"Joints : {len(joints)}")
    print(f"Frames : {len(joints_rotations)}")

    print("=================== Joints Hierarchy ===================")

    print_hierarchy(joints_hierarchy, joints[0])

    Draw_bvh(filename, joints, joints_offsets, joints_hierarchy, root_positions, joints_rotations, joints_saved_angles)
