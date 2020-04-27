if ENABLE_PLDM
utest_LDADD += \
	$(LIBPLDM_LIBS)

utest_SOURCES += \
	occ_reset_test.cpp

utest_CXXFLAGS += \
	$(LIBPLDM_CFLAGS)
endif
