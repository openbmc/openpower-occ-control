AM_CPPFLAGS = -I$(top_srcdir)
check_PROGRAMS = utest

# Run all 'check' test programs
TESTS = $(check_PROGRAMS)

utest_LDADD = $(top_builddir)/libocc_control.la -lstdc++fs

utest_SOURCES = utest.cpp \
                TestI2cOcc.cpp

utest_CPPFLAGS = $(GTEST_CPPFLAGS) \
                 $(AM_CPPFLAGS)

utest_CXXFLAGS = $(PTHREAD_CFLAGS)

utest_LDFLAGS = -lgmock_main \
                $(PTHREAD_LIBS) \
                $(OESDK_TESTCASE_FLAGS)
