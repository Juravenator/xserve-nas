{
  modulesPath,
  lib,
  pkgs,
  ...
}@args:
{
  imports = [
    ../modules/systemd-boot.nix
    ../modules/cicd.nix
    (modulesPath + "/installer/scan/not-detected.nix")
    (modulesPath + "/profiles/qemu-guest.nix")
  ];

  environment.systemPackages = map lib.lowPrio [
    pkgs.curl
    pkgs.gitMinimal
  ];

  services.openssh = {
    enable = true;
    ports = [ 666 ];
  };

  nix.settings.trusted-users = [
    "root"
    "@wheel"
    "cicd"
  ];
  nix.settings.experimental-features = [
    "nix-command"
    "flakes"
  ];

  users.users.root.openssh.authorizedKeys.keys = [
    "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIH4TfYrgjcXZd0q55fWHQqSwrGX8JwkF8kwUUNxj5wFA jura@juras-framework-12"
    "ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABAQC7Dn6N1vrwCLaHpMl60f5wifem27Cz/ZiH8/8zCwVYKVzKmKhu7DTlqu5Aark0OEvNLbWzzE3LBaO91l5yhuAZUB5lNZ3CIO/PfgKwux0szuY0xrMHToW/mTX04WJ60mGHKR23MacAEteInU1X6DBzjB/81yym/PxqezPy5iLtEvPicnwlbRcjqfxYu2CIxLbf0PPES+1G+zL1L8x1OYcE1k9gTnYtw6jyrBND8ZY9ach2Fu6cIveOs/gXQYCQw7Bq/Hsbr9Oobnurv/SiJjxmajFo46Ch+FVDRPeTWvONKqXu/ob8rtcojYKIT8r4lFBAl6pWx+7Ztdh+HROBHN8Z glenn@Glenns-MacBook-Pro.local"
    "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAICfKexmO90qYgyI4T53UH4vuGxRPFLbVYn8N+LP9AvOe jura@Glenns-MacBook-Pro.local"
  ]
  ++ (args.extraPublicKeys or [ ]); # this is used for unit-testing this module and can be removed if not needed
  security.sudo.wheelNeedsPassword = false;

  # This option defines the first version of NixOS you have installed on this particular machine,
  # and is used to maintain compatibility with application data (e.g. databases) created on older NixOS versions.
  #
  # Most users should NEVER change this value after the initial install, for any reason,
  # even if you've upgraded your system to a new NixOS release.
  #
  # This value does NOT affect the Nixpkgs version your packages and OS are pulled from,
  # so changing it will NOT upgrade your system - see https://nixos.org/manual/nixos/stable/#sec-upgrading for how
  # to actually do that.
  #
  # This value being lower than the current NixOS release does NOT mean your system is
  # out of date, out of support, or vulnerable.
  #
  # Do NOT change this value unless you have manually inspected all the changes it would make to your configuration,
  # and migrated your data accordingly.
  #
  # For more information, see `man configuration.nix` or https://nixos.org/manual/nixos/stable/options#opt-system.stateVersion .
  system.stateVersion = "25.11"; # Did you read the comment?
}
