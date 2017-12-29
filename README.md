## To Build
```
To build this package, do the following steps:

    1. ./bootstrap.sh
    2. ./configure ${CONFIGURE_FLAGS} --prefix=/usr
    3. make

To full clean the repository again run `./bootstrap.sh clean`.
```

## Supported Commands
- Partial Add
- Prepare for host update

```
For local build, pass --prefix=/usr option to the configure script
to let the Makefile use /usr/share value over /usr/local/share for ${datadir}
variable. The error yaml files and elog parser are stored in /usr/share
location in the SDK.
```
