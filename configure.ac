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

PKG_CHECK_MODULES([SDBUSPLUS], [sdbusplus],,\
    AC_MSG_ERROR(["Requires sdbusplus package."]))
PKG_CHECK_MODULES([PHOSPHOR_LOGGING], [phosphor-logging],,\
    AC_MSG_ERROR(["Requires phosphor-logging package."]))
PKG_CHECK_MODULES([OPENPOWER_DBUS_INTERFACES], [openpower-dbus-interfaces],,\
    AC_MSG_ERROR(["Requires openpower-dbus-interfaces package."]))

# Check for sdbusplus
PKG_CHECK_MODULES([SDBUSPLUS], [sdbusplus],, [AC_MSG_ERROR(["sdbusplus packaged required and not found"])])

# Check for sdbus++ tool
AC_PATH_PROG([SDBUSPLUSPLUS], [sdbus++])
AS_IF([test "x$SDBUSPLUSPLUS" == "x"], AC_MSG_ERROR([Cannot find sdbus++]))

# Checks for typedefs, structures, and compiler characteristics.
AX_CXX_COMPILE_STDCXX_14([noext])
AX_APPEND_COMPILE_FLAGS([-Wall -Werror], [CXXFLAGS])

AC_ARG_VAR(OCC_PASS_THROUGH_BUSNAME, [The Dbus busname to own])
AS_IF([test "x$OCC_PASS_THROUGH_BUSNAME" == "x"], [OCC_PASS_THROUGH_BUSNAME="org.open_power.OCC.PassThrough"])
AC_DEFINE_UNQUOTED([OCC_PASS_THROUGH_BUSNAME], ["$OCC_PASS_THROUGH_BUSNAME"], [The DBus busname to own])

AC_ARG_VAR(OCC_PASS_THROUGH_ROOT, [The Dbus root])
AS_IF([test "x$OCC_PASS_THROUGH_ROOT" == "x"], [OCC_PASS_THROUGH_ROOT="/xyz/openbmc_project/occ/pass_through"])
AC_DEFINE_UNQUOTED([OCC_PASS_THROUGH_ROOT], ["$OCC_PASS_THROUGH_ROOT"], [The Dbus root])

AC_ARG_VAR(INVENTORY_ITEM_INTERFACE, [The Inventory.Item interface])
AS_IF([test "x$INVENTORY_ITEM_INTERFACE" == "x"], [INVENTORY_ITEM_INTERFACE="xyz.openbmc_project.Inventory.Item"])
AC_DEFINE_UNQUOTED([INVENTORY_ITEM_INTERFACE], ["$INVENTORY_ITEM_INTERFACE"], [The Inventory.Item root])

AC_ARG_VAR(CPU_PATH, [CPU inventory path])
AS_IF([test "x$CPU_PATH" == "x"], [CPU_PATH="/xyz/openbmc_project/inventory/system/chassis/motherboard/cpu"])
AC_DEFINE_UNQUOTED([CPU_PATH], ["$CPU_PATH"], [CPU inventory path])

AC_ARG_VAR(MAX_CPUS, [The max number of CPUs])
AS_IF([test "x$MAX_CPUS" == "x"], [MAX_CPUS=2])
AC_DEFINE_UNQUOTED([MAX_CPUS], [$MAX_CPUS], [The max number of CPUs])

AC_CONFIG_HEADERS([config.h])
AC_CONFIG_FILES([Makefile])
AC_OUTPUT
