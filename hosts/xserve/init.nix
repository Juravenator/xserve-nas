{
  modulesPath,
  lib,
  pkgs,
  ...
} @ args:
{
  imports = [
    ./hardware-configuration.nix
    ./disk-config.nix
    ../../init/common.nix
  ];
}