# -*- mode: python ; coding: utf-8 -*-

import os

from PyInstaller.utils.hooks import collect_dynamic_libs, collect_submodules

_spec_dir = os.path.dirname(os.path.abspath(SPEC))
_root_dir = os.path.dirname(_spec_dir)
_src_dir = os.path.join(_root_dir, "src")
_entry = os.path.join(_src_dir, "robots_franka_fr3", "__main__.py")
_data_dir = os.path.join(_src_dir, "robots_franka_fr3", "data")

# franky-control 是 auditwheel 式自包含轮子：
#   - franky/_franky*.so（Python 扩展）与 franky/libfranky.so 位于包目录；
#   - 其余原生依赖（libfranka/libruckig/libfmt/libPoco*/libboost_*）位于
#     site-packages/franky_control.libs/，靠 RPATH $ORIGIN 解析。
# collect_dynamic_libs 收集包目录内的 libfranky.so；_franky 扩展由
# modulegraph 自动收集，其 ldd 依赖链（含 franky_control.libs 全部 .so）
# 由 PyInstaller 依赖分析自动平铺收集到 _MEIPASS 根目录。不要再手动
# glob franky_control.libs，否则与自动收集产生同路径重复条目，单文件
# 解压会冲突。运行时 bootloader 把 _MEIPASS 加入 LD_LIBRARY_PATH，
# 依赖均可被加载器找到。
binaries = collect_dynamic_libs("franky")

hiddenimports = (
    collect_submodules("robots_franka_fr3")
    + collect_submodules("franky")
    + collect_submodules("forge_msgs")
    + collect_submodules("forge_robot")
    + collect_submodules("forge_common")
    + [
        "dora",
        "pyarrow",
    ]
)

a = Analysis(
    [_entry],
    pathex=[_src_dir],
    binaries=binaries,
    datas=[
        (_data_dir, os.path.join("robots_franka_fr3", "data")),
    ],
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name="robots_franka_fr3",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
