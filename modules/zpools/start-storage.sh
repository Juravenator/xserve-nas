#!/usr/bin/env bash
set -o errexit -o nounset -o pipefail
IFS=$'\n\t\v'
cd `dirname "${BASH_SOURCE[0]:-$0}"`
# set -x

zpool import -a

read -sp 'Passphrase: ' ZFS_KEY
echo ""
echo "$ZFS_KEY" > /run/zfs.key

for dataset in "cold-2/vault" "hot-1/apps/owncloud" "hot-1/vault" "warm-1/vault"; do
  zfs load-key -r "$dataset"
  zfs mount -R "$dataset"
done

rm -f /run/zfs.key
