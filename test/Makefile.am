AM_CPPFLAGS = -I$(top_srcdir)
check_PROGRAMS = utest

# Run all 'check' test programs
TESTS = $(check_PROGRAMS)

# Build/add utest to test suite
utest_CPPFLAGS = -Igtest $(GTEST_CPPFLAGS) $(AM_CPPFLAGS)
utest_CXXFLAGS = $(PTHREAD_CFLAGS)
utest_LDFLAGS = -lgtest_main -lgtest \
                 $(PTHREAD_LIBS) \
                 $(OESDK_TESTCASE_FLAGS) \
                 $(SYSTEMD_LIBS) \
                 ${SDBUSPLUS_LIBS} \
                 $(OPENPOWER_DBUS_INTERFACES_LIBS) \
                 $(PHOSPHOR_DBUS_INTERFACES_LIBS) \
                 -lstdc++fs
utest_SOURCES = utest.cpp
utest_LDADD = $(top_builddir)/powercap.o \
              $(top_builddir)/occ_status.o \
              $(top_builddir)/occ_device.o \
              $(top_builddir)/occ_errors.o \
              $(top_builddir)/i2c_occ.o \
              $(top_builddir)/utils.o \
              $(top_builddir)/org/open_power/OCC/Device/error.o
