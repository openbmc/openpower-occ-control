project(
    'openpower-occ-control', 'cpp',
    version : '1.0.0',
    meson_version: '>=1.1.1',
    default_options: [
        'warning_level=3',
        'werror=true',
        'cpp_std=c++23',
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
conf_data.set_quoted('OCC_CONTROL_PERSIST_PATH', '/var/lib/openpower-occ-control')

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

install_data('occ-active.sh',
    install_mode: 'rwxr-xr-x',
    install_dir: get_option('bindir')
)

systemd = dependency('systemd')
systemd_system_unit_dir = systemd.get_variable(
    'systemdsystemunitdir')
subdir('service_files')

sdbusplus_dep = dependency('sdbusplus')

python_prog = find_program('python3', required: true)

deps = []
sources = []
if get_option('install-error-yaml').disabled()
    sdeventplus_dep = dependency(
        'sdeventplus',
        fallback: [
            'sdeventplus',
            'sdeventplus_dep'
        ],
    )
    phosphor_dbus_interfaces_dep = dependency(
        'phosphor-dbus-interfaces',
        fallback: [
            'phosphor-dbus-interfaces',
            'phosphor_dbus_interfaces_dep'
        ],
    )
    phosphor_logging_dep = dependency(
        'phosphor-logging',
        fallback: [
            'phosphor-logging',
            'phosphor_logging_dep'
        ],
    )

    if cxx.has_header('nlohmann/json.hpp')
        nlohmann_json_dep = declare_dependency()
    else
        subproject('nlohmann-json')
        nlohmann_json_dep = declare_dependency(
            include_directories: [
                'subprojects/nlohmann-json/single_include',
                'subprojects/nlohmann-json/single_include/nlohmann',
            ]
        )
    endif

    deps += [
        nlohmann_json_dep,
        phosphor_dbus_interfaces_dep,
        phosphor_logging_dep,
        sdbusplus_dep,
        sdeventplus_dep,
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
        include_directories: '.',
        implicit_include_directories: true,
        dependencies: deps,
        install: true,
        install_dir: get_option('bindir')
    )
endif

if not get_option('tests').disabled()
  subdir('test')
endif
