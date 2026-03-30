{ name, config, lib, pkgs, modulesPath, ... }:

let
  mkSyncoid = import ./make-syncoid.nix { inherit lib pkgs; };
in {
  environment.systemPackages = with pkgs; [
    smartmontools
    sanoid
  ];

  environment.etc."check-zpool-disks-active.sh" = {
    mode = "0755";
    source = ./sanoid/check-zpool-disks-active.sh;
  };

  systemd.services.sanoid = {
    serviceConfig = {
      DynamicUser = lib.mkForce false;
      User = lib.mkForce "root";
      Group = lib.mkForce "root";
      ExecStartPre = lib.mkForce [];
      ExecStopPost = lib.mkForce [];

      NoNewPrivileges = "yes";
      PrivateTmp = "yes";
      ProtectSystem = "strict";
      ProtectHome = "yes";
      RestrictAddressFamilies = "AF_UNIX";
    };
  };

  services.sanoid = {
    enable = true;
    # https://www.freedesktop.org/software/systemd/man/latest/systemd.time.html#Calendar%20Events
    interval = "*-*-* *:00:00"; # hourly

    templates = {
      "short-term" = {
        autosnap = true;
        autoprune = true;
        pre_snapshot_script = "/etc/check-zpool-disks-active.sh";

        hourly = 30;
        daily = 10;
        weekly = 4;
        monthly = 0;
        yearly = 0;
      };
      "long-term" = {
        autosnap = true;
        autoprune = true;
        pre_snapshot_script = "/etc/check-zpool-disks-active.sh";

        hourly = 30;
        daily = 10;
        weekly = 6;
        monthly = 15;
        yearly = 5;
      };
    };

    datasets = {
        "hot-1" = {
          useTemplate = ["long-term"];
          recursive = true;
        };
        "warm-1/rips" = {
          useTemplate = ["short-term"];
          recursive = true;
        };
        "warm-1/vault" = {
          useTemplate = ["short-term"];
          recursive = true;
        };
        "cold-1" = {
          useTemplate = ["long-term"];
          recursive = true;
        };
        "cold-2" = {
          useTemplate = ["long-term"];
          recursive = true;
        };
    };
  };

  imports = [
    (mkSyncoid "*-*-* 08,12,16,20:01:00" "hot-1" "warm-1/replicas/hot-1")
    (mkSyncoid "*-*-* 08,12,16,20:01:00" "hot-1" "cold-1/replicas/hot-1")
    (mkSyncoid "*-*-* 08,12,16,20:01:00" "warm-1/rips" "cold-1/replicas/warm-1/rips")
    (mkSyncoid "*-*-* 08,12,16,20:01:00" "warm-1/vault" "cold-1/replicas/warm-1/vault")
  ];

}