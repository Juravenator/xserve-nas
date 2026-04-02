{ name, config, lib, pkgs, modulesPath, ... }:

let
  check-zpool-active = ./sanoid/check-zpool-disks-active.sh;
  mkScrub = import ./make-scrub.nix { inherit lib pkgs check-zpool-active; };
in {
  imports = [
    (mkScrub "hot-1" "7 days" "14 days")
    (mkScrub "warm-1" "7 days" "30 days")
    (mkScrub "cold-1" "30 days" "60 days")
    (mkScrub "cold-2" "30 days" "60 days")
    (mkScrub "zroot" "7 days" "14 days")
  ];
}