{ config, lib, pkgs, modulesPath, ... }:

{
  imports = [ ];

  # environment.etc."cicd.sh" = {
  #   source = ./cicd.sh;
  #   mode = "0755";
  # };

  users.users.cicd = {
    isNormalUser = true;
    extraGroups = ["wheel"];
    openssh.authorizedKeys.keys = [
      "no-port-forwarding,no-X11-forwarding,no-agent-forwarding ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAICsg1Dc8IEen4T8n7sgL+mPzBmA0TpWHn3/CNX7F4oaC CICD"
    ];
  };
}
