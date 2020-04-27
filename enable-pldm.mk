if ENABLE_PLDM
openpower_occ_control_LDADD += \
	$(LIBPLDM_LIBS)

openpower_occ_control_CXXFLAGS += \
	$(LIBPLDM_CFLAGS)
endif
