gtest_dep = dependency('gtest', main: true, disabler: true, required: false)
gmock_dep = dependency('gmock', disabler: true, required: false)
if not gtest_dep.found() or not gmock_dep.found()
    gtest_proj = import('cmake').subproject('googletest', required: false)
    if gtest_proj.found()
        gtest_dep = declare_dependency(
            dependencies: [
                dependency('threads'),
                gtest_proj.dependency('gtest'),
                gtest_proj.dependency('gtest_main'),
            ],
        )
        gmock_dep = gtest_proj.dependency('gmock')
    else
        assert(
            not get_option('tests').allowed(),
            'Googletest is required if tests are enabled',
        )
    endif
endif

test_sources = [
    '../occ_errors.cpp',
    '../occ_ffdc.cpp',
    '../occ_presence.cpp',
    '../occ_command.cpp',
    '../occ_dbus.cpp',
    '../powercap.cpp',
    '../i2c_occ.cpp',
    '../utils.cpp',
    occ_sensor_hpp,
]

if get_option('power10-support').allowed()
    test_sources += [
        '../pldm.cpp']
        '../powermode.cpp'
        '../occ_pass_through.cpp',
        '../occ_manager.cpp',
        '../occ_status.cpp',
        '../occ_device.cpp',
    ]
else
    test_sources += [
        '../legacy/occ_pass_through.cpp',
        '../legacy/occ_manager.cpp',
        '../legacy/occ_status.cpp',
        '../legacy/occ_device.cpp',
    ]
endif

tests = [
    'error_files_tests.cpp',
    'occ_dbus_test.cpp',
    'TestI2cOcc.cpp',
    'utest.cpp',
]

pthread_dep = dependency('threads')

foreach t : tests
    test(
        t,
        executable(
            t.underscorify(),
            t,
            test_sources,
            include_directories: '..',
            dependencies: [gtest_dep, gmock_dep, pthread_dep, deps],
        ),
        workdir: meson.current_source_dir(),
    )
endforeach
