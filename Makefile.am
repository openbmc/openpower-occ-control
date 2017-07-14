# Build these headers, don't install them
noinst_HEADERS = \
	occ_pass_through.hpp \
	occ_status.hpp

noinst_LTLIBRARIES = libocc_control.la

sbin_PROGRAMS = openpower-occ-control

openpower_occ_control_SOURCES = \
	occ_pass_through.cpp \
	occ_status.cpp \
	occ_device.cpp \
	app.cpp \
	org/open_power/OCC/PassThrough/error.cpp

libocc_control_la_SOURCES = utils.cpp

openpower_occ_control_LDADD = libocc_control.la -lstdc++fs

BUILT_SOURCES =  org/open_power/OCC/PassThrough/error.hpp \
                 org/open_power/OCC/PassThrough/error.cpp

CLEANFILES = ${BUILT_SOURCES}

generic_cxx_flags = \
	$(SDBUSPLUS_CFLAGS) \
	$(PHOSPHOR_LOGGING_CFLAGS) \
	$(OPENPOWER_DBUS_INTERFACES_CFLAGS)

generic_ld_flags = \
	$(SDBUSPLUS_LIBS) \
	$(PHOSPHOR_LOGGING_LIBS) \
	$(OPENPOWER_DBUS_INTERFACES_LIBS)

libocc_control_la_CXXFLAGS = $(generic_cxx_flags)

libocc_control_la_LDFLAGS = $(generic_ld_flags)

openpower_occ_control_CXXFLAGS = $(generic_cxx_flags)

openpower_occ_control_LDFLAGS = $(generic_ld_flags)

org/open_power/OCC/PassThrough/error.hpp: ${top_srcdir}/org/open_power/OCC/PassThrough.errors.yaml
	@mkdir -p `dirname $@`
	$(SDBUSPLUSPLUS) -r $(top_srcdir) error exception-header org.open_power.OCC.PassThrough > $@

org/open_power/OCC/PassThrough/error.cpp: ${top_srcdir}/org/open_power/OCC/PassThrough.errors.yaml
	@mkdir -p `dirname $@`
	$(SDBUSPLUSPLUS) -r $(top_srcdir) error exception-cpp org.open_power.OCC.PassThrough > $@

SUBDIRS = . test
