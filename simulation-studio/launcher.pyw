"""Desktop entry point with local diagnostics if startup fails."""
from pathlib import Path
import os
import sys
import traceback
import ctypes

root=Path(__file__).resolve().parent
os.chdir(root)
with (root/'launcher.log').open('a',encoding='utf-8',buffering=1) as log:
    sys.stdout=sys.stderr=log
    try:
        from app import main
        main()
    except Exception:
        traceback.print_exc()
        ctypes.windll.user32.MessageBoxW(None,'程序启动失败。请查看项目 simulation-studio 文件夹中的 launcher.log。','TME180 Simulation Studio',0x10)
