{ name, config, lib, pkgs, modulesPath, ... }:

let
  lcars-dashboard = pkgs.stdenv.mkDerivation {
    pname = "lcars-dashboard";
    version = "0.1.8";

    src = pkgs.fetchzip {
      url = "https://github.com/juravenator/lcars-dashboard/releases/download/v0.1.8/dashboard-api-x86_64-unknown-linux-musl.tar.xz";
      sha256 = "sha256-iB9MdYuKxObkWqgWKm9C9Ew0NqIWX217kLDNy9EdcNs=";
    };

    installPhase = ''
      mkdir -p $out/bin
      cp dashboard-api $out/bin/
      chmod +x $out/bin/dashboard-api
    '';
  };
in
{
  environment.systemPackages = [ lcars-dashboard ];

  systemd.services.lcars-dashboard = {
    description = "LCARS Dashboard";
    wantedBy = [ "multi-user.target" ];

    path = [
      pkgs.zfs
      pkgs.coreutils
      pkgs.gnugrep
      pkgs.smartmontools
    ];

    serviceConfig = {
      ExecStart = "${lcars-dashboard}/bin/dashboard-api";
      Restart = "always";
      Environment = "KUBECONFIG=/etc/rancher/k3s/k3s.yaml";
    };
  };

  networking.firewall.allowedTCPPorts = [ 3000 ];
}