AM_CPPFLAGS = -I$(top_srcdir)
check_PROGRAMS = utest

# Run all 'check' test programs
TESTS = $(check_PROGRAMS)

utest_LDADD = \
	$(top_builddir)/libocc_control.la \
	-lstdc++fs \
	-lgmock \
	$(SDBUSPLUS_LIBS) \
	$(SDEVENTPLUS_LIBS) \
	$(FMT_LIBS)

utest_SOURCES = error_files_tests.cpp \
                utest.cpp \
                TestI2cOcc.cpp \
                occ_dbus_test.cpp

utest_CPPFLAGS = $(GTEST_CPPFLAGS) \
                 $(AM_CPPFLAGS)

utest_CXXFLAGS = $(PTHREAD_CFLAGS)

utest_LDFLAGS = -lgtest -lgmock -lgmock_main \
                $(PTHREAD_LIBS) \
                $(OESDK_TESTCASE_FLAGS)
