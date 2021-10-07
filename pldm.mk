if ENABLE_PLDM

noinst_HEADERS += \
	pldm.hpp
libocc_control_la_SOURCES += \
	pldm.cpp
openpower_occ_control_LDADD += \
	$(LIBPLDM_LIBS) $(PDBG_LIBS) -lphal
openpower_occ_control_CXXFLAGS += \
	$(LIBPLDM_CFLAGS) $(PDBG_CFLAGS)

endif
