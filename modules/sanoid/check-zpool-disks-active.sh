#!/usr/bin/env bash
set -o errexit -o nounset -o pipefail
IFS=$'\n\t\v,'

if [[ -z "${1:-}" ]]; then
    >&2 echo "Usage $0 <pool>"
    exit 1
fi

for pool in $1; do
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
