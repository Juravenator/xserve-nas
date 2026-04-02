{ lib, pkgs, check-zpool-active }:

pool:

{
  environment.etc."sanoid/${pool}/sanoid.conf" = {
    mode = "0644";
    source = ./${pool}.conf;
  };

  systemd.tmpfiles.rules = [
    "d /var/cache/sanoid/${pool} 0755 root root -"
  ];

  systemd.services."sanoid-${pool}" = {
    after = [ "zfs.target" ];
    startAt = "*-*-* *:00:00";

    description = "Sanoid snapshot service for ${pool}";

    path = [ "/run/current-system/sw" ];
    script = ''
      #!/usr/bin/env bash
      set -o errexit -o nounset -o pipefail
      IFS=$'\n\t\v'
      set -x

      if ! ${check-zpool-active}/bin/check-zpool-active ${pool}; then
        exit 0
      fi

      ${pkgs.sanoid}/bin/sanoid --cron \
        --configdir /etc/sanoid/${pool} \
        --cache-dir /var/cache/sanoid/${pool} \
    '';
  };
}