#!/usr/bin/env python3
"""Source preparation for the official CubeMX static-library workflow on Lyrical."""
import concurrent.futures
import json
from pathlib import Path
import shutil
import subprocess
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
IGNORE = [
    'ros2/rcl', 'ros2/rcl_logging/rcl_logging_spdlog',
    'ros2/rcl_logging/rcl_logging_implementation',
    'ros2/rosidl/rosidl_typesupport_introspection_cpp', 'ros2/rcpputils',
    'ros2/ros2_tracing/test_tracetools', 'uros/rcl/rcl_yaml_param_parser',
    'uros/rclc/rclc_examples', 'ros2/ros2_tracing/lttngpy',
    'ros2/rosidl/rosidl_buffer', 'ros2/rosidl/rosidl_buffer_backend',
    'ros2/rosidl/rosidl_buffer_backend_registry', 'ros2/rmw/rmw_security_common',
]


def checkout(item):
    name, source = item
    dest = ROOT / '.cache/client_ws' / name
    dest.mkdir(parents=True, exist_ok=True)
    if not (dest / '.git').exists():
        subprocess.run(['git', 'init', '-q', str(dest)], check=True)
        subprocess.run(['git', '-C', str(dest), 'remote', 'add', 'origin', source['url']], check=True)
    current = subprocess.run(['git', '-C', str(dest), 'rev-parse', 'HEAD'],
                             capture_output=True, text=True).stdout.strip()
    if current != source['version']:
        subprocess.run(['git', '-C', str(dest), 'fetch', '-q', '--depth', '1',
                        'origin', source['version']], check=True)
        subprocess.run(['git', '-C', str(dest), 'checkout', '-q', '--detach',
                        source['version']], check=True)


def prepare():
    sources = yaml.safe_load((ROOT / 'micro_ros/client.repos').read_text())['repositories']
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        list(pool.map(checkout, sources.items()))
    workspace = ROOT / '.cache/client_ws'
    (workspace / 'ros2/rcl_logging/COLCON_IGNORE').unlink(missing_ok=True)
    for name in IGNORE:
        if (workspace / name).is_dir():
            (workspace / name / 'COLCON_IGNORE').touch()
    interfaces = ROOT.parents[1] / 'interfaces/minesweeper_interfaces'
    shutil.copytree(interfaces, workspace / 'extra_packages/minesweeper_interfaces', dirs_exist_ok=True)
    # Host tools remain native Python/CMake. Hide installed host RMW and DDS
    # generators from CMake/ament when building Cortex-M libraries.
    host_share = Path('/opt/ros/lyrical/share')
    tools = ROOT / '.cache/host_tools/share'
    tools.mkdir(parents=True, exist_ok=True)
    names = {p.name for p in host_share.glob('ament*') if p.name != 'ament_index'}
    names.update(['rosidl_adapter', 'rosidl_parser', 'rosidl_cmake',
                  'rosidl_pycommon', 'rosidl_generator_c', 'rosidl_cli',
                  'rosidl_generator_type_description', 'rosidl_typesupport_interface',
                  'rosidl_runtime_cmake'])
    for name in names:
        if (host_share / name).exists() and not (tools / name).exists():
            (tools / name).symlink_to(host_share / name, target_is_directory=True)
    for kind in (host_share / 'ament_index/resource_index').iterdir():
        if not kind.is_dir():
            continue
        for entry in kind.iterdir():
            if entry.name in names:
                out = tools / 'ament_index/resource_index' / kind.name / entry.name
                out.parent.mkdir(parents=True, exist_ok=True)
                if not out.exists():
                    out.symlink_to(entry)


def stage():
    install = ROOT / '.cache/client_ws/install'
    dest = ROOT / 'micro_ros/libmicroros'
    shutil.rmtree(dest, ignore_errors=True)
    shutil.copytree(install / 'include', dest / 'include')
    # MRI CREATE/ADDLIB preserves members with matching basenames, unlike ar x.
    libs = sorted((install / 'lib').rglob('*.a'))
    if not libs:
        raise RuntimeError('No cross-compiled archives found')
    script = 'CREATE ' + str(dest / 'libmicroros.a') + '\n'
    script += ''.join('ADDLIB ' + str(p) + '\n' for p in libs) + 'SAVE\nEND\n'
    import os
    subprocess.run([os.environ.get('TOOLCHAIN_PREFIX', 'arm-none-eabi-') + 'ar', '-M'],
                   input=script, text=True, check=True)
    provenance = {'client_manifest': 'client.repos', 'ros_distro': 'lyrical',
                  'setup_commit': 'e64204663fb734e24b5de3cd04fb03d3188468d9',
                  'utils_commit': '3eb71ecebd57c166f0f4367cc2b0b1c4fe4b3234'}
    (dest / 'provenance.json').write_text(json.dumps(provenance, indent=2) + '\n')
    # Narrow patches to the pinned official custom heap: preserve integration
    # while fixing null calloc, arithmetic overflow and realloc(p,0).
    src = (ROOT / 'vendor/utils/extra_sources/custom_memory_manager.c').read_text()
    src = src.replace('#include <stdlib.h>', '#include <stdlib.h>\n#include <stdint.h>')
    src = src.replace('ucHeap[ configTOTAL_HEAP_SIZE ]', 'ucHeap[ configMICROROS_HEAP_SIZE ]')
    src = src.replace('xTotalHeapSize = configTOTAL_HEAP_SIZE', 'xTotalHeapSize = configMICROROS_HEAP_SIZE')
    src = src.replace('void *pvPortCallocMicroROS( size_t num, size_t xWantedSize )\n{',
        'void *pvPortCallocMicroROS( size_t num, size_t xWantedSize )\n{\n'
        '\tif (num && xWantedSize > SIZE_MAX / num) return NULL;')
    src = src.replace('char *in_dest = (char*)mem;',
        'if (!mem) { (void)xTaskResumeAll(); return NULL; }\n\tchar *in_dest = (char*)mem;')
    src = src.replace('void *pvPortReallocMicroROS( void *pv, size_t xWantedSize )\n{',
        'void *pvPortReallocMicroROS( void *pv, size_t xWantedSize )\n{\n'
        '\tif (!xWantedSize) { vPortFreeMicroROS(pv); return NULL; }')
    src = src.replace('void *pvPortMallocMicroROS( size_t xWantedSize )\n{',
        'void *pvPortMallocMicroROS( size_t xWantedSize )\n{\n'
        '\tif (xWantedSize > SIZE_MAX - xHeapStructSize - portBYTE_ALIGNMENT) return NULL;')
    generated = ROOT / 'micro_ros/generated'
    generated.mkdir(exist_ok=True)
    (generated / 'custom_memory_manager.c').write_text(src)
    time_src = (ROOT / 'vendor/utils/extra_sources/microros_time.c').read_text()
    time_src = time_src.replace('clock_gettime( int clock_id,', 'clock_gettime( clockid_t clock_id,')
    (generated / 'microros_time.c').write_text(time_src)


if __name__ == '__main__':
    stage() if '--stage' in sys.argv else prepare()
