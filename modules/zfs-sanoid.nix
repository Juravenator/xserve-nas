{ name, config, lib, pkgs, modulesPath, ... }:

let
  check-zpool-active = pkgs.writeShellScriptBin "check-zpool-active" (
    builtins.readFile ./sanoid/check-zpool-disks-active.sh
  );

  mkSyncoid = import ./sanoid/make-syncoid.nix { inherit lib pkgs check-zpool-active; };
  mkSnap = import ./sanoid/make-snap.nix { inherit lib pkgs check-zpool-active; };
in {
  environment.systemPackages = with pkgs; [
    smartmontools
    sanoid
    check-zpool-active
  ];

  imports = [
    (mkSnap "hot-1")
    (mkSnap "warm-1")
    (mkSnap "cold-1")
    (mkSnap "cold-2")

    (mkSyncoid "*-*-* 08,12,16,20:01:00" "hot-1" "warm-1/replicas/hot-1")
    (mkSyncoid "*-*-* 08,12,16,20:01:00" "hot-1" "cold-1/replicas/hot-1")
    (mkSyncoid "*-*-* 08,12,16,20:01:00" "warm-1/rips" "cold-1/replicas/warm-1/rips")
    (mkSyncoid "*-*-* 08,12,16,20:01:00" "warm-1/vault" "cold-1/replicas/warm-1/vault")
  ];

}