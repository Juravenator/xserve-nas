{ lib, pkgs, check-zpool-active }:

pool: int_min: int_max:

{
  systemd.services."scrub-${pool}" = {
    after = [ "zfs.target" ];
    startAt = "*-*-* *:00:00";

    path = [ "/run/current-system/sw" ];
    script = ''
      #!/usr/bin/env bash
      set -o errexit -o nounset -o pipefail
      IFS=$'\n\t\v'

      scrub_status=$(zpool status -j --json-int ${pool} | jq -r '.pools["${pool}"].scan_stats.state')

      if [[ "$scrub_status" == "SCANNING" ]]; then
        echo "scrub is still busy"
      fi

      scrub_end=$(zpool status -j --json-int ${pool} | jq '.pools["${pool}"].scan_stats.end_time')
      if [[ "$scrub_end" == "null" ]]; then
        scrub_end=0
      fi
      scrub_min_age=$(date +%s -d '${int_min} ago')
      scrub_max_age=$(date +%s -d '${int_max} ago')

      echo "should trigger if min_age ($(date -d @$scrub_min_age '+%FT%R')) > last_scrub ($(date -d @$scrub_end '+%FT%R')) or last_scrub ($(date -d @$scrub_end '+%FT%R')) > max_age ($(date -d @$scrub_max_age '+%FT%R'))"

      if [[ $scrub_end -gt $scrub_min_age ]]; then
        echo "last scrub is not older than minimum age"
        exit 0
      fi

      if [[ $scrub_end -lt $scrub_max_age ]]; then
        echo "last scrub is older than max age. scrub should be started even if disks are sleeping"
      else
        echo "scrub should be scheduled, but is not past max age."
        if ! ${check-zpool-active}/bin/check-zpool-active ${pool}; then
          echo "not all disks are active, will wait till they are"
          exit 0
        fi
      fi

      ${pkgs.zfs}/bin/zpool scrub ${pool}
      ${pkgs.zfs}/bin/zpool status ${pool}
    '';
  };
}