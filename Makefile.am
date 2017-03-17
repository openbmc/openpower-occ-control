# Build these headers, don't install them
noinst_HEADERS = \
	occ_pass_through.hpp

sbin_PROGRAMS = openpower-occ-control
openpower_occ_control_SOURCES = \
	app.cpp

openpower_occ_control_LDFLAGS = \
	$(SDBUSPLUS_LIBS) \
	$(PHOSPHOR_LOGGING_LIBS) \
	$(OPENPOWER_DBUS_INTERFACES_LIBS)
openpower_occ_control_CXXFLAGS =
	$(SDBUSPLUS_CFLAGS) \
	$(PHOSPHOR_LOGGING_CFLAGS) \
	$(OPENPOWER_DBUS_INTERFACES_CFLAGS)
