# openpower-occ-control
On chip control to keep the sytem under power/thermal limits.

## To Build
```
To build this package, do the following steps:

    1. ./bootstrap.sh
    2. ./configure ${CONFIGURE_FLAGS} --prefix=/usr
    3. make
To clean the repository run `./bootstrap.sh clean`.
```

## Supported Commands
- Partial Add
- Prepare for host update

For local builds, pass the `--prefix=/usr` option to the configure script to
have the Makefile use `/usr/share` over `/usr/local/share` for the ${datadir}
variable. The error YAML files and elog parser are stored in the SDK at
`/usr/share`.
