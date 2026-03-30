#!/usr/bin/env bash
set -o errexit -o nounset -o pipefail
IFS=$'\n\t\v,'

SANOID_TARGETS=${SANOID_TARGETS:-$1}

echo "SANOID_TARGETS=$SANOID_TARGETS"

for pool in $SANOID_TARGETS; do
    pool=${pool%%/*}
    echo "checking pool $pool"
    for disk_path in $(zpool status -PL $pool | grep -o '/dev/[a-zA-Z0-9/\-]*'); do
        dev=$(lsblk -no PKNAME $disk_path)
        echo -n "  checking disk $dev ($disk_path)"
        ! /run/current-system/sw/bin/smartctl -n standby -i $disk_path &> /dev/null
        if [[ ${PIPESTATUS[0]} -ne 0 ]]; then
            echo " IDLE"
            echo "not all disks are active"
            exit 2
        else
            echo " ACTIVE"
        fi
    done
done
