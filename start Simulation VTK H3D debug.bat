@echo off 
@set CL=/D_CRT_SECURE_NO_DEPRECATE /D_CRT_NONSTDC_NO_DEPRECATE
@set LINK=/LARGEADDRESSAWARE

set HAPIDir=K:\lib-x64-msvc2019\h3d\hapi-1.4.0\build\Debug
set H3DUtilDir=K:\lib-x64-msvc2019\h3d\h3dutil-1.4.0\build\Debug
set PthreadsDir=K:\lib-x64-msvc2019\pthreads\pthreads-w32-2-9-1-release\Pre-built.2\dll\x64
set VTKDir=K:\lib-x64-msvc2019\VTK-9.0.1-build\bin\Debug
set VirtuoseDir=K:\lib-x64-msvc2019\Virtuose\VirtuoseAPI_v3_97\win\bin\VC2017\x64\Debug
set ZLibDir=K:\lib-x64-msvc2019\Anaconda\Anaconda3\pkgs\zlib-1.2.11-h62dcd97_4\Library\bin
PATH=%HAPIDir%;%H3DUtilDir%;%PthreadsDir%;%VTKDir%;%VirtuoseDir%;%ZLibDir%;%PATH%

Debug\Simulation_d.exe

