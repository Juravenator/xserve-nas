{ lib, pkgs, check-zpool-active }:

interval: source: target:

let
  # https://github.com/NixOS/nixpkgs/blob/nixos-unstable/nixos/modules/services/backup/syncoid.nix#L20C3-L25C7
  escapeUnitName =
    name:
    # join pieces, or return input if there was nothing to split
    lib.concatMapStrings (s: if lib.isList s then "-" else s) (
      # split into pieces of valid characters
      builtins.split "[^a-zA-Z0-9_.\\-]+" name
    );
in {
  systemd.services."syncoid-${escapeUnitName source}_${escapeUnitName target}" = {
    after = [ "zfs.target" ];
    startAt = interval;

    path = [ "/run/current-system/sw" ];
    script = ''
      #!/usr/bin/env bash
      set -o errexit -o nounset -o pipefail
      IFS=$'\n\t\v'
      set -x
      id
      which zpool

      ${check-zpool-active}/bin/check-zpool-active ${source}
      ${check-zpool-active}/bin/check-zpool-active ${target}

      ${pkgs.sanoid}/bin/syncoid -r --no-sync-snap --force-delete ${lib.escapeShellArgs [source target]}
    '';
  };
}