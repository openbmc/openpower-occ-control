# Build these headers, don't install them
noinst_HEADERS = \
	occ_pass_through.hpp

sbin_PROGRAMS = openpower-occ-control
openpower_occ_control_SOURCES = \
	occ_pass_through.cpp \
	app.cpp \
	org/open_power/OCC/PassThrough/error.cpp

BUILT_SOURCES =  org/open_power/OCC/PassThrough/error.hpp \
                 org/open_power/OCC/PassThrough/error.cpp

CLEANFILES = ${BUILT_SOURCES}

openpower_occ_control_LDFLAGS = \
	$(SDBUSPLUS_LIBS) \
	$(PHOSPHOR_LOGGING_LIBS) \
	$(OPENPOWER_DBUS_INTERFACES_LIBS) \
        -lstdc++fs

openpower_occ_control_CXXFLAGS =
	$(SDBUSPLUS_CFLAGS) \
	$(PHOSPHOR_LOGGING_CFLAGS) \
	$(OPENPOWER_DBUS_INTERFACES_CFLAGS)

org/open_power/OCC/PassThrough/error.hpp: ${top_srcdir}/org/open_power/OCC/PassThrough.errors.yaml
	@mkdir -p `dirname $@`
	$(SDBUSPLUSPLUS) -r $(top_srcdir) error exception-header org.open_power.OCC.PassThrough > $@

org/open_power/OCC/PassThrough/error.cpp: ${top_srcdir}/org/open_power/OCC/PassThrough.errors.yaml
	@mkdir -p `dirname $@`
	$(SDBUSPLUSPLUS) -r $(top_srcdir) error exception-cpp org.open_power.OCC.PassThrough > $@

SUBDIRS = . status
