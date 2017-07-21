AC_PREREQ([2.69])
AC_INIT([openpower-occ-control], [1.0], [https://github.com/openbmc/openpower-occ-control/issues])

AM_INIT_AUTOMAKE([subdir-objects -Wall -Werror foreign dist-xz])
AM_SILENT_RULES([yes])

# Checks for programs
AC_PROG_CXX
AM_PROG_AR
AC_PROG_INSTALL
AC_PROG_MAKE_SET

# Surpress the --with-libtool-sysroot error
LT_INIT

# gtest
# Check/set gtest specific functions.
AX_PTHREAD([GTEST_CPPFLAGS="-DGTEST_HAS_PTHREAD=1"],[GTEST_CPPFLAGS="-DGTEST_HAS_PTHREAD=0"])
AC_SUBST(GTEST_CPPFLAGS)

# Test cases require SDK so only build if we're told to (and SDK is available)
AC_ARG_ENABLE([oe-sdk],
    AS_HELP_STRING([--enable-oe-sdk], [Link testcases absolutely against OE SDK so they can be ran within it.])
)
AC_ARG_VAR(OECORE_TARGET_SYSROOT,
    [Path to the OE SDK SYSROOT])
AS_IF([test "x$enable_oe_sdk" == "xyes"],
    AS_IF([test "x$OECORE_TARGET_SYSROOT" == "x"],
          AC_MSG_ERROR([OECORE_TARGET_SYSROOT must be set with --enable-oe-sdk])
    )
    AC_MSG_NOTICE([Enabling OE-SDK at $OECORE_TARGET_SYSROOT])
    [
        testcase_flags="-Wl,-rpath,\${OECORE_TARGET_SYSROOT}/lib"
        testcase_flags="${testcase_flags} -Wl,-rpath,\${OECORE_TARGET_SYSROOT}/usr/lib"
        testcase_flags="${testcase_flags} -Wl,-dynamic-linker,`find \${OECORE_TARGET_SYSROOT}/lib/ld-*.so | sort -r -n | head -n1`"
    ]
    AC_SUBST([OESDK_TESTCASE_FLAGS], [$testcase_flags])
)

PKG_CHECK_MODULES([SDBUSPLUS], [sdbusplus],,\
    AC_MSG_ERROR(["Requires sdbusplus package."]))
PKG_CHECK_MODULES([PHOSPHOR_LOGGING], [phosphor-logging],,\
    AC_MSG_ERROR(["Requires phosphor-logging package."]))
PKG_CHECK_MODULES([OPENPOWER_DBUS_INTERFACES], [openpower-dbus-interfaces],,\
    AC_MSG_ERROR(["Requires openpower-dbus-interfaces package."]))
PKG_CHECK_MODULES([PHOSPHOR_DBUS_INTERFACES], [phosphor-dbus-interfaces],,\
    AC_MSG_ERROR(["Requires phosphor-dbus-interfaces package."]))

# Check for sdbusplus
PKG_CHECK_MODULES([SDBUSPLUS], [sdbusplus],, [AC_MSG_ERROR(["sdbusplus packaged required and not found"])])

# Check for sdbus++ tool
AC_PATH_PROG([SDBUSPLUSPLUS], [sdbus++])
AS_IF([test "x$SDBUSPLUSPLUS" == "x"], AC_MSG_ERROR([Cannot find sdbus++]))

# Checks for typedefs, structures, and compiler characteristics.
AX_CXX_COMPILE_STDCXX_14([noext])
AX_APPEND_COMPILE_FLAGS([-Wall -Werror], [CXXFLAGS])

AC_ARG_VAR(OCC_CONTROL_BUSNAME, [The Dbus busname to own])
AS_IF([test "x$OCC_CONTROL_BUSNAME" == "x"], [OCC_CONTROL_BUSNAME="org.open_power.OCC.Control"])
AC_DEFINE_UNQUOTED([OCC_CONTROL_BUSNAME], ["$OCC_CONTROL_BUSNAME"], [The DBus busname to own])

AC_ARG_VAR(OCC_CONTROL_ROOT, [The Dbus root])
AS_IF([test "x$OCC_CONTROL_ROOT" == "x"], [OCC_CONTROL_ROOT="/org/open_power/control"])
AC_DEFINE_UNQUOTED([OCC_CONTROL_ROOT], ["$OCC_CONTROL_ROOT"], [The Dbus root])

AC_ARG_VAR(INVENTORY_ITEM_INTERFACE, [The Inventory.Item interface])
AS_IF([test "x$INVENTORY_ITEM_INTERFACE" == "x"], [INVENTORY_ITEM_INTERFACE="xyz.openbmc_project.Inventory.Item"])
AC_DEFINE_UNQUOTED([INVENTORY_ITEM_INTERFACE], ["$INVENTORY_ITEM_INTERFACE"], [The Inventory.Item root])

AC_ARG_VAR(CPU_PATH, [CPU inventory path])
AS_IF([test "x$CPU_PATH" == "x"], [CPU_PATH="/xyz/openbmc_project/inventory/system/chassis/motherboard/cpu"])
AC_DEFINE_UNQUOTED([CPU_PATH], ["$CPU_PATH"], [CPU inventory path])

AC_ARG_VAR(MAX_CPUS, [The max number of CPUs])
AS_IF([test "x$MAX_CPUS" == "x"], [MAX_CPUS=2])
AC_DEFINE_UNQUOTED([MAX_CPUS], [$MAX_CPUS], [The max number of CPUs])

AC_ARG_VAR(CPU_NAME, [The CPU object name])
AS_IF([test "x$CPU_NAME" == "x"], [CPU_NAME="cpu"])
AC_DEFINE_UNQUOTED([CPU_NAME], ["$CPU_NAME"], [The CPU object name])

AC_ARG_VAR(OCC_NAME, [The OCC object name])
AS_IF([test "x$OCC_NAME" == "x"], [OCC_NAME="occ"])
AC_DEFINE_UNQUOTED([OCC_NAME], ["$OCC_NAME"], [The OCC object name])

AC_ARG_VAR(OCC_MASTER_NAME, [The OCC master object name])
AS_IF([test "x$OCC_MASTER_NAME" == "x"], [OCC_MASTER_NAME="occ1-dev0"])
AC_DEFINE_UNQUOTED([OCC_MASTER_NAME], ["$OCC_MASTER_NAME"], [The OCC master object name])

AC_ARG_VAR(OCC_HWMON_PATH, [The OCC hwmon path])
AS_IF([test "x$OCC_HWMON_PATH" == "x"], [OCC_HWMON_PATH="/sys/bus/platform/drivers/occ-hwmon/"])
AC_DEFINE_UNQUOTED([OCC_HWMON_PATH], ["$OCC_HWMON_PATH"], [The OCC hwmon path])

AC_ARG_VAR(DEV_PATH, [The device path])
AS_IF([test "x$DEV_PATH" == "x"], [DEV_PATH="/sys/bus/platform/devices/"])
AC_DEFINE_UNQUOTED([DEV_PATH], ["$DEV_PATH"], [The device path])

AC_ARG_VAR(PS_DERATING_FACTOR, [The power supply derating factor])
AS_IF([test "x$PS_DERATING_FACTOR" == "x"], [PS_DERATING_FACTOR=90])
AC_DEFINE_UNQUOTED([PS_DERATING_FACTOR], [$PS_DERATING_FACTOR], [The power supply derating factor])

AC_ARG_VAR(CPU_SUBPATH, [The subpath containing CPU objects])
AS_IF([test "x$CPU_SUBPATH" == "x"], [CPU_SUBPATH="/xyz/openbmc_project/inventory/system/chassis/motherboard"])
AC_DEFINE_UNQUOTED([CPU_SUBPATH], ["$CPU_SUBPATH"], [The subpath containing CPU objects])

# Handling configuration files
AC_ARG_VAR(YAML_PATH, [The path to the yaml config files.])
AS_IF([test "x$YAML_PATH" == "x"], [YAML_PATH="$srcdir/example"])
AC_DEFINE_UNQUOTED([YAML_PATH], ["$YAML_PATH"], [The path to the yaml config files])

AC_CONFIG_HEADERS([config.h])
AC_CONFIG_FILES([Makefile test/Makefile])
AC_OUTPUT
