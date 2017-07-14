AM_CPPFLAGS = -I${top_srcdir}
check_PROGRAMS =

TESTS = $(check_PROGRAMS)

check_PROGRAMS += test

test_SOURCES = \
    TestUtils.cpp

test_CPPFLAGS = $(GTEST_CPPFLAGS) \
                $(AM_CPPFLAGS)

test_CXXFLAGS = $(PTHREAD_CFLAGS)

test_LDFLAGS = -lgmock_main \
               $(PTHREAD_LIBS) \
               $(OESDK_TESTCASE_FLAGS) \
               -lstdc++fs

