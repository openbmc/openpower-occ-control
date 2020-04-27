AM_CPPFLAGS = -I$(top_srcdir)
check_PROGRAMS = utest

# Run all 'check' test programs
TESTS = $(check_PROGRAMS)

utest_LDADD = $(top_builddir)/libocc_control.la -lstdc++fs \
              $(LIBPLDM_LIBS)

utest_SOURCES = error_files_tests.cpp \
                utest.cpp \
                TestI2cOcc.cpp \
                occ_reset_test.cpp

utest_CPPFLAGS = $(GTEST_CPPFLAGS) \
                 $(AM_CPPFLAGS)

utest_CXXFLAGS = $(PTHREAD_CFLAGS) \
                 $(LIBPLDM_CFLAGS)

utest_LDFLAGS = -lgtest -lgmock -lgmock_main \
                $(PTHREAD_LIBS) \
                $(OESDK_TESTCASE_FLAGS)

#include enable-pldm-utest.mk
