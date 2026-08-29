{ name, config, lib, pkgs, modulesPath, ... }: {
  imports = [];

  services.openssh = {
    enable = true;
    ports = [ 666 ];
    settings = {
      PasswordAuthentication = false;
      KbdInteractiveAuthentication = false;
      PermitRootLogin = "no";
    };
  };
  services.fail2ban = {
    enable = true;
  };
  services.endlessh-go = {
    enable = true;
    port = 22;
    # So that we can have port 22 for gitea on zt
    listenAddress = "192.168.42.133";
  };

  networking.firewall.allowedTCPPorts = [ 22 666 ];
  networking.firewall.enable = true;
}
