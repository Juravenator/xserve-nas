{
  config,
  lib,
  pkgs,
  modulesPath,
  ...
}:

let
  sanoid-shell = pkgs.writeShellScriptBin "floere-sanoid-restricted-shell" ''
    #!/usr/bin/env bash
    set -o errexit -o nounset -o pipefail

    cmd="$1"
    shift 2>/dev/null || true

    if [[ -z "$cmd" ]]; then
      echo "Interactive shell is not allowed" >&2
      exit 1
    fi

    valid_dataset="hot-1/vault/cowtea"

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
    if [[ "$@" == "  zfs send "* ]] && [[ "$@" == *"$valid_dataset"* ]]; then
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

  users.users.floere = {
    isNormalUser = true;
    shell = "${sanoid-shell}/bin/floere-sanoid-restricted-shell";
    useDefaultShell = false;
    openssh = {
      authorizedKeys = {
        keys = [
          "no-port-forwarding,no-X11-forwarding,no-agent-forwarding,no-user-rc,restrict ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIHojDNIrtqi9L1sCfqVYrhjpB+Pf3SFKDmfP3q2vVcJm"
        ];
      };
    };
  };
}
