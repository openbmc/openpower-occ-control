unit_files = [
   'org.open_power.OCC.Control.service',
   'op-occ-disable@.service',
   'op-occ-enable@.service'
]

foreach u : unit_files
    configure_file(
        copy: true,
        input: u,
        install: true,
        install_dir: systemd_system_unit_dir,
        output: u,
        )
endforeach
