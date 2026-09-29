"""ctypes bindings for the bundled esmini 3.8.1 ABI (double precision)."""
import ctypes as C
import math
import os
from settings import RUNTIME

class State(C.Structure):
    _fields_ = [(k, C.c_int) for k in ['id', 'model_id', 'ctrl_type']] + [
        (k, C.c_double) for k in ['timestamp', 'x', 'y', 'z', 'h', 'p', 'r']] + [
        ('roadId', C.c_uint32), ('junctionId', C.c_uint32), ('t', C.c_double),
        ('laneId', C.c_int)] + [(k, C.c_double) for k in [
        'laneOffset', 's', 'speed', 'centerOffsetX', 'centerOffsetY', 'centerOffsetZ',
        'width', 'length', 'height']] + [('objectType', C.c_int), ('objectCategory', C.c_int),
        ('wheel_angle', C.c_double), ('wheel_rot', C.c_double), ('visibilityMask', C.c_int)]

class Image(C.Structure):
    _fields_ = [(k, C.c_int) for k in ['width', 'height', 'pixelSize', 'pixelFormat']] + [('data', C.POINTER(C.c_ubyte))]

class Engine:
    def __init__(self):
        self.sensors = {}
        self.dll_dir = os.add_dll_directory(str(RUNTIME / 'bin'))
        self.lib = C.CDLL(str(RUNTIME / 'bin/esminiLib.dll'))
        signatures = {
            'SE_InitWithArgs': ([C.c_int, C.POINTER(C.c_char_p)], C.c_int),
            'SE_AddPath': ([C.c_char_p], C.c_int),
            'SE_GetNumberOfObjects': ([], C.c_int), 'SE_GetId': ([C.c_int], C.c_int),
            'SE_GetObjectName': ([C.c_int], C.c_char_p),
            'SE_GetObjectState': ([C.c_int, C.POINTER(State)], C.c_int),
            'SE_GetObjectVelocityGlobalXYZ': ([C.c_int] + [C.POINTER(C.c_double)] * 3, C.c_int),
            'SE_AddObjectSensor': ([C.c_int] + [C.c_double] * 7 + [C.c_int], C.c_int),
            'SE_FetchSensorObjectList': ([C.c_int, C.POINTER(C.c_int)], C.c_int),
            'SE_GetObjectCollision': ([C.c_int, C.c_int], C.c_int),
            'SE_GetObjectNumberOfCollisions': ([C.c_int], C.c_int),
            'SE_GetSimulationTime': ([], C.c_double), 'SE_GetQuitFlag': ([], C.c_int),
            'SE_GetODRFilename': ([], C.c_char_p),
            'SE_ReportObjectPos': ([C.c_int] + [C.c_double] * 6, C.c_int),
            'SE_ReportObjectSpeed': ([C.c_int, C.c_double], C.c_int),
            'SE_ReportObjectVel': ([C.c_int] + [C.c_double] * 3, C.c_int),
            'SE_StepDT': ([C.c_double], C.c_int), 'SE_Close': ([], None),
            'SE_SaveImagesToRAM': ([C.c_bool], C.c_int),
            'SE_FetchImage': ([C.POINTER(Image)], C.c_int),
        }
        for name, (args, ret) in signatures.items():
            f = getattr(self.lib, name)
            f.argtypes, f.restype = args, ret

    def init(self, scenario, logfile, *, dat=None, video=False, camera=None):
        for p in [RUNTIME / 'resources', RUNTIME / 'resources/models', RUNTIME / 'resources/xodr']:
            self.lib.SE_AddPath(str(p).encode())
        args = ['esmini', '--osc', str(scenario), '--headless', '--collision',
                '--seed', '1', '--logfile_path', str(logfile), '--disable_stdout']
        if dat:
            args += ['--record', str(dat)]
        if video:
            args += ['--window', '0', '0', '960', '540', '--camera_mode', 'custom' if camera else 'top', '--info_text', '1', '--ground_plane', '--disable_shadows']
            if camera:
                args += ['--custom_fixed_top_camera', ','.join(map(str,camera))]
            self.lib.SE_SaveImagesToRAM(True)
        self.args = args
        argv = (C.c_char_p * len(args))(*[v.encode('utf-8') for v in args])
        if self.lib.SE_InitWithArgs(len(args), argv) != 0:
            raise RuntimeError('esmini 初始化失败，请查看 esmini.log。')

    def states(self):
        return [self.object_state(self.lib.SE_GetId(index))
                for index in range(self.lib.SE_GetNumberOfObjects())]

    def object_state(self, ident):
        state = State()
        if self.lib.SE_GetObjectState(ident, C.byref(state)) != 0:
            raise RuntimeError('无法读取对象状态')
        d = {k: getattr(state, k) for k, _ in State._fields_}
        d['name'] = self.lib.SE_GetObjectName(ident).decode('utf-8')
        vx, vy, vz = C.c_double(), C.c_double(), C.c_double()
        if self.lib.SE_GetObjectVelocityGlobalXYZ(ident, C.byref(vx), C.byref(vy), C.byref(vz)) != 0:
            raise RuntimeError('无法读取对象速度')
        d.update(vx=vx.value, vy=vy.value, vz=vz.value)
        return d

    def add_object_sensor(self, ego, config):
        # v3.8.1's native hit buffer is not capped during Update(). Reserve room
        # for every entity, and reject population growth before any later step.
        capacity = max(1, self.lib.SE_GetNumberOfObjects())
        mount = [ego['centerOffsetX'], ego['centerOffsetY'], ego['centerOffsetZ']]
        ident = self.lib.SE_AddObjectSensor(ego['id'], *mount, 0.0, 0.0,
                                            config['sensor_range_m'],
                                            math.radians(config['sensor_fov_deg']), capacity)
        if ident < 0:
            raise RuntimeError('esmini 理想目标传感器创建失败')
        self.sensors[ident] = {
            'backend': 'esmini_ideal_object_sensor', 'sensor_id': ident,
            'host_object_id': ego['id'], 'mount_xyz_m': mount, 'heading_rad': 0.0,
            'near_m': 0.0, 'far_m': config['sensor_range_m'],
            'horizontal_fov_deg': config['sensor_fov_deg'], 'capacity': capacity,
            'detection_geometry': 'Native horizontal gate from ego bounding-box centre to target reference point.',
            'measurement_geometry': 'Detected object state converted to bounding-box centre-to-centre range before uncertainty.',
            'uncertainty_stage': 'After native detection; only detected targets draw range error.',
            'limitations': 'Ideal object-level detection; no occlusion, physical sensor response or velocity noise.',
            'fetch_count': 0,
        }
        return ident

    def detect(self, sensor_id):
        sensor = self.sensors[sensor_id]
        ids = (C.c_int * sensor['capacity'])()
        count = self.lib.SE_FetchSensorObjectList(sensor_id, ids)
        if not 0 <= count <= sensor['capacity']:
            raise RuntimeError('esmini 传感器探测查询失败')
        sensor['fetch_count'] += 1
        # Stable ID order also makes the uncertainty draws deterministic.
        return [self.object_state(ident) for ident in sorted(ids[:count])]

    def report(self, d):
        if self.lib.SE_ReportObjectPos(d['id'], *[d[k] for k in ['x', 'y', 'z', 'h', 'p', 'r']]) != 0:
            raise RuntimeError('报告对象位置失败')
        self.lib.SE_ReportObjectSpeed(d['id'], d['speed'])
        if 'vx' in d:
            self.lib.SE_ReportObjectVel(d['id'], d['vx'], d['vy'], 0)

    def step(self, dt):
        if any(self.lib.SE_GetNumberOfObjects() > s['capacity'] for s in self.sensors.values()):
            raise RuntimeError('场景对象数量超出 esmini 传感器缓冲容量')
        if self.lib.SE_StepDT(dt) != 0:
            raise RuntimeError('esmini 步进失败')

    def collisions(self, ident):
        count = self.lib.SE_GetObjectNumberOfCollisions(ident)
        if count < 0:
            raise RuntimeError('碰撞查询失败')
        return [self.lib.SE_GetObjectCollision(ident, i) for i in range(count)]

    def image(self):
        im = Image()
        if self.lib.SE_FetchImage(C.byref(im)) != 0 or not im.data:
            raise RuntimeError('视频图像捕获失败')
        if im.pixelSize != 3 or im.pixelFormat not in [0x1907, 0x80E0]:
            raise RuntimeError('不支持的图像格式')
        return im.width, im.height, ('rgb24' if im.pixelFormat == 0x1907 else 'bgr24'), C.string_at(im.data, im.width * im.height * 3)

    def close(self):
        self.lib.SE_Close()
