{ name, config, lib, pkgs, modulesPath, ... }:

{
  imports = [ ];

  # https://discourse.nixos.org/t/my-zfs-volumes-fail-to-mount-at-boot/5445
  # The proposed fix of disabling zfs-mount.service does not work.
  systemd.services.zroot-legacy = {
    description = "Ensure zroot mountpoint is set to legacy";
    after = [ "multi-user.target" ];
    wantedBy = [ "multi-user.target" ];
    serviceConfig = {
      Type = "oneshot";
    };
    script = ''
      if ${pkgs.zfs}/bin/zfs get -H mountpoint zroot | grep -v legacy; then
        ${pkgs.zfs}/bin/zfs set -u mountpoint=legacy zroot
      fi
    '';
  };
}