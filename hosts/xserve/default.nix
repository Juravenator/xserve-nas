{ name, nodes, pkgs, lib, ... }: {

    deployment = {
        targetHost = "xserve.nest.jurabytes.net";
        # targetHost = "192.168.42.133";
        targetPort = 666;
        # targetUser = "cicd";
        targetUser = "jura";
        buildOnTarget = true;
    };

    imports = [
        ./hardware-configuration.nix
        ../../modules/zpools.nix
        ../../modules/zfs-fix.nix
        ../../modules/systemd-boot.nix
        ../../modules/basics.nix
        ../../modules/sshd.nix
        ../../modules/cicd.nix
        ../../modules/users.nix
        ../../modules/hardening.nix
        ../../modules/k3s.nix
        ../../modules/zerotier.nix
        ../../modules/samba.nix
        ../../modules/dashboard.nix
        ../../modules/zfs-sanoid.nix
        ../../modules/zfs-scrub.nix
        ../../modules/fluufff-zfs.nix
    ];

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
