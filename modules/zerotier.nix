{ config, lib, pkgs, modulesPath, ... }:

{
  imports = [ ];

  services.zerotierone = {
    enable = true;
    joinNetworks = [
      "272f5eae168fe329"
    ];
  };
}
