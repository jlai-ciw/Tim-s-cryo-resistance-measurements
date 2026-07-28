# -*- mode: python ; coding: utf-8 -*-

from PyInstaller.utils.hooks import copy_metadata

# pyvisa finds the pyvisa-py backend at runtime via importlib.metadata entry
# points, not a normal import -- PyInstaller's static analysis can't see that,
# so without these the frozen exe bundles neither the pyvisa_py module nor
# the package metadata pyvisa needs to discover it, and every VISA connect
# fails with "Could not locate a VISA implementation."
datas = copy_metadata('pyvisa') + copy_metadata('pyvisa-py')

a = Analysis(
    ['cryo_resistance_logger.py'],
    pathex=[],
    binaries=[],
    datas=datas,
    hiddenimports=['pyvisa_py'],
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
    [],
    exclude_binaries=True,
    name='Cryo_resistance',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
# onedir build: unlike onefile, this doesn't re-extract the whole bundle to a
# temp folder on every launch -- that extraction step is the likely cause of
# the slow/hanging startup on older hardware.
coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='Cryo_resistance',
)
