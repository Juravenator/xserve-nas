{ config, lib, pkgs, modulesPath, ... }:

{
  imports = [ ];

  services.zerotierone = {
    joinNetworks = [
      "68bea79acfeca5af"
    ];
  };

  users.users.pawhost-test = {
    isNormalUser = true;
    openssh = {
      authorizedKeys = {
        keys = [
          "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIBETdBupEy69aDwqcKpfI7Om31k7iVbnJGJ9AhGgC2eB"
        ];
      };
    };
  };
}
