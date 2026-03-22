{ lib, pkgs }:

user: url:

let
  serviceName = "${user}-ssh-keys";
in {
  users.users.${user} = {
    isNormalUser = true;
    extraGroups = ["wheel" "data"];
  };

  systemd.timers.${serviceName} = {
    wantedBy = [ "timers.target" ];
    timerConfig = {
      OnCalendar = "daily";
      OnBootSec = "1sec";
      Persistent = true;
      Unit = "${serviceName}.service";
    };
  };

  systemd.services.${serviceName} = {
    script = ''
      #!/usr/bin/env bash
      set -o errexit -o nounset -o pipefail
      IFS=$'\n\t\v'
      mkdir -p $HOME/.ssh
      ${pkgs.curl}/bin/curl '${url}' > $HOME/.ssh/authorized_keys
    '';
    serviceConfig = {
      Type = "oneshot";
      User = user;
    };
  };
}