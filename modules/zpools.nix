{ name, config, lib, pkgs, modulesPath, ... }:

{
  imports = [ ];

  environment.etc."start-storage.sh" = {
    source = ./zpools/start-storage.sh;
    mode = "0744";
  };
}