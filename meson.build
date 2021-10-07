project(
    'openpower-occ-control', 'cpp',
    version : '1.0.0',
    meson_version: '>=0.57.0',
    default_options: [
        'warning_level=3',
        'werror=true',
        'cpp_std=c++20',
        'buildtype=debugoptimized'
    ]
)

cxx = meson.get_compiler('cpp')

conf_data = configuration_data()
conf_data.set_quoted('OCC_CONTROL_BUSNAME', 'org.open_power.OCC.Control')
conf_data.set_quoted('OCC_CONTROL_ROOT', '/org/open_power/control')
conf_data.set_quoted('OCC_SENSORS_ROOT', '/xyz/openbmc_project/sensors')
conf_data.set_quoted('CPU_NAME', 'cpu')
conf_data.set_quoted('OCC_NAME', 'occ')
conf_data.set_quoted('OCC_MASTER_NAME', 'occ-hwmon.1')
conf_data.set_quoted('OCC_DEV_PATH', '/dev/occ')
conf_data.set_quoted('CPU_SUBPATH', '/xyz/openbmc_project/inventory/system/chassis/motherboard')

conf_data.set('MAX_CPUS', get_option('max-cpus'))
conf_data.set('OCC_CPU_TEMP_SENSOR_TYPE', 0xC0)
conf_data.set('OCC_DIMM_TEMP_SENSOR_TYPE', 0xD0)
conf_data.set('PS_DERATING_FACTOR', get_option('ps-derating-factor'))

if get_option('i2c-occ').enabled()
    conf_data.set_quoted('OCC_HWMON_PATH', '/sys/bus/i2c/drivers/occ-hwmon/')
    conf_data.set_quoted('DEV_PATH', '/sys/bus/i2c/devices')
    conf_data.set_quoted('I2C_OCC_DEVICE_NAME', 'p8-occ-hwmon')
else
    conf_data.set_quoted('OCC_HWMON_PATH', '/sys/bus/platform/drivers/occ-hwmon/')
    conf_data.set_quoted('DEV_PATH', '/sys/bus/platform/devices/')
endif

if get_option('install-error-yaml').disabled()
    conf_data.set('I2C_OCC', get_option('i2c-occ').enabled())
    conf_data.set('READ_OCC_SENSORS', get_option('read-occ-sensors').enabled())
    conf_data.set('PLDM', get_option('with-host-communication-protocol')=='pldm')
    conf_data.set('POWER10', get_option('power10-support').enabled())
endif

configure_file(output: 'config.h',
    configuration: conf_data
)

sdbusplusplus_prog = find_program('sdbus++')
sdbuspp_gen_meson_prog = find_program('sdbus++-gen-meson')
python_prog = find_program('python3', required: true)
realpath_prog = find_program('realpath')

selected_subdirs = []
selected_subdirs += 'org/open_power/OCC'

generated_root = meson.current_build_dir() / 'gen'
generated_others = []
generated_sources = []

# Source the generated meson files.
subdir('gen')
foreach d : selected_subdirs
  subdir('gen' / d)
endforeach

# Parse through the list from sdbus++-gendir and put into sets.
generated_headers = []
generated_cpp = []
generated_others_files = []

foreach g : generated_sources generated_others
    foreach f : g.to_list()
        rel_path = run_command(
            realpath_prog,
            '--relative-to', generated_root,
            f.full_path(),
        ).stdout().strip().split('\n')[-1]

        if rel_path.endswith('.hpp')
            generated_headers += rel_path
        elif rel_path.endswith('.cpp')
            generated_cpp += rel_path
        else
            generated_others_files += rel_path
        endif
    endforeach
endforeach

deps = []
sources = []
if get_option('install-error-yaml').disabled()
    sdbusplus_dep = dependency('sdbusplus')
    if sdbusplus_dep.found()
        sdbusplusplus_prog = find_program('sdbus++')
        sdbuspp_gen_meson_prog = find_program('sdbus++-gen-meson')
    else
        sdbusplus_proj = subproject('sdbusplus', required: true)
        sdbusplus_dep = sdbusplus_proj.get_variable('sdbusplus_dep')
        sdbusplusplus_prog = sdbusplus_proj.get_variable('sdbusplusplus_prog')
        sdbuspp_gen_meson_prog = sdbusplus_proj.get_variable(
            'sdbuspp_gen_meson_prog'
        )
    endif

    sdeventplus_dep = dependency(
        'sdeventplus',
        fallback: [
            'sdeventplus',
            'sdeventplus_dep'
        ],
    )
    phosphor_logging_dep = dependency(
        'phosphor-logging',
        fallback: [
            'phosphor-logging',
            'phosphor_logging_dep'
        ],
    )
    phosphor_dbus_interfaces_dep = dependency(
        'phosphor-dbus-interfaces',
        fallback: [
            'phosphor-dbus-interfaces',
            'phosphor_dbus_interfaces_dep'
        ],
    )

    deps += [
        sdbusplus_dep,
        sdeventplus_dep,
        phosphor_logging_dep,
        phosphor_dbus_interfaces_dep,
    ]

    sources += [
        'app.cpp',
        'occ_pass_through.cpp',
        'occ_manager.cpp',
        'occ_status.cpp',
        'occ_device.cpp',
        'occ_errors.cpp',
        'occ_ffdc.cpp',
        'occ_presence.cpp',
        'occ_command.cpp',
        'occ_dbus.cpp',
        'powercap.cpp',
        'i2c_occ.cpp',
        'utils.cpp',
    ]

    if get_option('with-host-communication-protocol')=='pldm'
        libpldm_dep = dependency(
            'libpldm',
            fallback: ['pldm', 'libpldm_dep'],
            default_options: ['libpldm-only=enabled', 'oem-ibm=enabled'],
        )
        deps += [
            libpldm_dep,
            cxx.find_library('pdbg'),
            cxx.find_library('phal'),
        ]
        sources += [
            'pldm.cpp',
        ]
    endif

    if get_option('power10-support').enabled()
        sources += [
            'powermode.cpp',
        ]
    endif

    yamldir = get_option('yamldir')
    if yamldir == ''
        yamldir = meson.project_source_root() / 'example'
    endif

    # Generate occ-sensor.hpp.
    occ_sensor_hpp = custom_target(
        'occ-sensor.hpp',
        command : [
            python_prog,
            meson.project_source_root() + '/sensor_gen.py',
            '-i', yamldir,
        ],
        output : 'occ-sensor.hpp')
    sources += [occ_sensor_hpp]

    executable(
        'openpower-occ-control',
        sources,
        generated_sources,
        include_directories: ['.', 'gen'],
        implicit_include_directories: true,
        dependencies: deps,
        install: true,
        install_dir: get_option('bindir')
    )
endif

if not get_option('tests').disabled()
  subdir('test')
endif
