# Build these headers, don't install them
noinst_HEADERS = \
	occ_pass_through.hpp \
	occ_status.hpp \
	powercap.hpp \
	occ_errors.hpp \
	occ_events.hpp \
	occ_finder.hpp \
	utils.hpp

sbin_PROGRAMS = openpower-occ-control
openpower_occ_control_SOURCES = \
	occ_pass_through.cpp \
	occ_status.cpp \
	occ_device.cpp \
	occ_errors.cpp \
	app.cpp \
	powercap.cpp \
	org/open_power/OCC/Device/error.cpp \
	occ_finder.cpp \
	utils.cpp

BUILT_SOURCES =  org/open_power/OCC/Device/error.hpp \
                 org/open_power/OCC/Device/error.cpp \
                 occ_sensor.hpp

CLEANFILES = ${BUILT_SOURCES}

REQ_MAKO_FILE ?= ${top_srcdir}/occ_sensor.mako.hpp
REQ_PY_SCRIPT ?= ${top_srcdir}/sensor_gen.py

EXTRA_DIST = $(REQ_MAKO_FILE) $(REQ_PY_SCRIPT)

occ_sensor.hpp: ${REQ_PY_SCRIPT} ${REQ_MAKO_FILE}
	$(AM_V_GEN) ${PYTHON} ${REQ_PY_SCRIPT} -i ${YAML_PATH} > $@

openpower_occ_control_LDFLAGS = \
	$(SDBUSPLUS_LIBS) \
	$(PHOSPHOR_LOGGING_LIBS) \
	$(OPENPOWER_DBUS_INTERFACES_LIBS) \
	$(PHOSPHOR_DBUS_INTERFACES_LIBS) \
        -lstdc++fs

openpower_occ_control_CXXFLAGS =
	$(SDBUSPLUS_CFLAGS) \
	$(PHOSPHOR_LOGGING_CFLAGS) \
	$(OPENPOWER_DBUS_INTERFACES_CFLAGS) \
	$(PHOSPHOR_DBUS_INTERFACES_CFLAGS)

org/open_power/OCC/Device/error.hpp: ${top_srcdir}/org/open_power/OCC/Device.errors.yaml
	@mkdir -p `dirname $@`
	$(SDBUSPLUSPLUS) -r $(top_srcdir) error exception-header org.open_power.OCC.Device > $@

org/open_power/OCC/Device/error.cpp: ${top_srcdir}/org/open_power/OCC/Device.errors.yaml
	@mkdir -p `dirname $@`
	$(SDBUSPLUSPLUS) -r $(top_srcdir) error exception-cpp org.open_power.OCC.Device > $@

SUBDIRS = . test
