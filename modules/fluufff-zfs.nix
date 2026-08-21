{
  config,
  lib,
  pkgs,
  modulesPath,
  ...
}:

let
  sanoid-shell = pkgs.writeShellScriptBin "sanoid-restricted-shell" ''
    #!/usr/bin/env bash
    set -o errexit -o nounset -o pipefail

    cmd="$1"
    shift 2>/dev/null || true

    if [[ -z "$cmd" ]]; then
      echo "Interactive shell is not allowed" >&2
      exit 1
    fi

    valid_dataset="hot-1/replicas/test.pawhost.fluufff.org"

    allowed=false

    if [[ "$@" == "exit" ]] ||
       [[ "$@" == "echo "* ]] ||
       [[ "$@" == "ps "* ]] ||
       [[ "$@" == "command "* ]]
    then
      allowed=true
    fi

    if [[ "$@" == "zfs get "* ]] && [[ "$@" == *"$valid_dataset"* ]]; then
      allowed=true
    fi
    if [[ "$@" == "  zfs receive "* ]] && [[ "$@" == *"$valid_dataset"* ]]; then
      allowed=true
    fi

    if [[ "$@" == "zpool get "* ]] && [[ "$@" == *"hot-1"* ]]; then
      allowed=true
    fi

    if [[ "$allowed" == "true" ]]; then
      bash -c "$@"
    else
      AUDIT_FILE="/tmp/pawhost-test-audit.log"
      echo "$(date '+%FT%R'): '$cmd' '$@'" >> "$AUDIT_FILE"
      echo "This command is not allowed"
      exit 1
    fi
  '';
in

{
  imports = [ ];

  services.zerotierone = {
    joinNetworks = [
      "68bea79acfeca5af"
    ];
  };

  users.users.pawhost-test = {
    isNormalUser = true;
    shell = "${sanoid-shell}/bin/sanoid-restricted-shell";
    useDefaultShell = false;
    openssh = {
      authorizedKeys = {
        keys = [
          "no-port-forwarding,no-X11-forwarding,no-agent-forwarding,no-user-rc,restrict ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIBETdBupEy69aDwqcKpfI7Om31k7iVbnJGJ9AhGgC2eB"
        ];
      };
    };
  };
}
