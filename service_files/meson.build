unit_files = [
    'org.open_power.OCC.Control.service',
    'op-occ-disable@.service',
    'op-occ-enable@.service',
]

fs = import('fs')
foreach u : unit_files
    fs.copyfile(u, install: true, install_dir: systemd_system_unit_dir)
endforeach
