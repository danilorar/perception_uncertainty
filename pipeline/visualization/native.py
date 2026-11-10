"""Small public esmini 3.8.1 graphics API client. No experiment controller runs here."""
import ctypes as C
import os
from pathlib import Path


class CapturedImage(C.Structure):
    _fields_ = [('width', C.c_int), ('height', C.c_int), ('pixelSize', C.c_int),
                ('pixelFormat', C.c_int), ('data', C.POINTER(C.c_ubyte))]


class Engine:
    def __init__(self, runtime):
        self.runtime = Path(runtime).resolve()
        name = 'esminiLib.dll' if os.name == 'nt' else 'libesminiLib.dylib'
        path = self.runtime / 'bin' / name
        if not path.is_file():
            raise ValueError(f'Graphics library not found: {path}. Use --visual-runtime with a graphics-enabled esmini 3.8.1 installation.')
        self.dll_dir = os.add_dll_directory(str(path.parent)) if os.name == 'nt' else None
        self.lib = C.CDLL(str(path))
        signatures = {
            'SE_InitWithArgs': ([C.c_int, C.POINTER(C.c_char_p)], C.c_int),
            'SE_AddPath': ([C.c_char_p], C.c_int),
            'SE_SaveImagesToRAM': ([C.c_bool], C.c_int),
            'SE_ReportObjectPos': ([C.c_int] + [C.c_double]*6, C.c_int),
            'SE_ReportObjectSpeed': ([C.c_int, C.c_double], C.c_int),
            'SE_ReportObjectVel': ([C.c_int] + [C.c_double]*3, C.c_int),
            'SE_AddCustomFixedTopCamera': ([C.c_double]*4, C.c_int),
            'SE_SetCameraMode': ([C.c_int], C.c_int),
            'SE_StepDT': ([C.c_double], C.c_int),
            'SE_FetchImage': ([C.POINTER(CapturedImage)], C.c_int),
            'SE_Close': ([], None),
        }
        for name, (args, result) in signatures.items():
            function = getattr(self.lib, name)
            function.argtypes, function.restype = args, result
        self.initialized = False

    def init(self, scene, logfile, *, camera, size):
        for suffix in ['resources', 'resources/models', 'resources/xodr']:
            self.lib.SE_AddPath(str(self.runtime / suffix).encode('utf-8'))
        self.lib.SE_SaveImagesToRAM(True)
        args = ['esmini', '--osc', str(scene), '--window', '0', '0', *map(str, size),
                '--camera_mode', 'custom', '--custom_fixed_top_camera', ','.join(map(str, camera)),
                '--info_text', '0', '--ground_plane', '--disable_shadows', '--traj_filter', '0',
                '--logfile_path', str(logfile), '--disable_stdout']
        if os.name == 'nt':
            args += ['--headless']
        # On macOS the hidden pbuffer captures black frames; a normal graphics
        # context is needed for video. The experiment itself remains headless.
        argv = (C.c_char_p * len(args))(*[value.encode('utf-8') for value in args])
        if self.lib.SE_InitWithArgs(len(args), argv):
            raise RuntimeError('Graphics replay initialization failed; see the replay log.')
        self.initialized = True

    def report(self, state):
        if self.lib.SE_ReportObjectPos(state['id'], *[state[key] for key in ['x', 'y', 'z', 'h', 'p', 'r']]):
            raise RuntimeError('Cannot report recorded vehicle position.')
        self.lib.SE_ReportObjectSpeed(state['id'], state['speed'])
        self.lib.SE_ReportObjectVel(state['id'], state['vx'], state['vy'], 0)

    def step(self, dt):
        if self.lib.SE_StepDT(dt):
            raise RuntimeError('Graphics replay step failed.')

    def image(self):
        image = CapturedImage()
        if self.lib.SE_FetchImage(C.byref(image)) or not image.data:
            raise RuntimeError('No rendered image captured.')
        if image.pixelSize != 3 or image.pixelFormat not in (0x1907, 0x80E0):
            raise RuntimeError('Unsupported captured pixel format.')
        return (image.width, image.height, 'rgb24' if image.pixelFormat == 0x1907 else 'bgr24',
                C.string_at(image.data, image.width*image.height*3))

    def close(self):
        if self.initialized:
            self.lib.SE_Close()
            self.initialized = False
        if self.dll_dir:
            self.dll_dir.close()
