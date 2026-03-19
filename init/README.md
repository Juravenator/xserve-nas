# Nixos-anywhere init flake

This flake can be used to remotely initialize a NixOS installation on the server.

It only bothers with disk formatting, base os installation, and access for the CI/CD user.
Colmena is to handle the rest in a CI/CD pipeline.

> [!CAUTION]
> This reformats the disk(s) on the remote machines.
> If this host is in use, there will be data loss.

## Initializing a new host

### 1. Create a bare host to install to.

Boot any linux you want, can be nixos, debian, anything.
It does not need to be installed, it can be a live CD.
The only thing you need is SSH root or sudo-passwordless access.

### 2. Populate the new host config

Create files `hosts/<name>/default.nix`, `hosts/<name>/disk-config.nix`, `hosts/<name>/init.nix`, and add `<name>` in the host entries in `flake.nix`.
Use the existing hosts for examples.

### 3. Generate hardware config

You will need to retrieve a hardware config and put it in `hosts/<name>/hardware-configuration.nix`.

On a nixos install, you will find this at `/etc/nixos/hardware-configuration.nix`.

If this file doesn't exist, or you are not using nixos as a bootstrap os, we can generate one ourselves.

On a nixos install:
```
nixos-generate-config --show-hardware-config
```

On a linux install:
- make sure nix is installed (https://nix.dev/install-nix)
- `sudo nix-shell -p nixos-install-tools`
- `nixos-generate-config --root /mnt --show-hardware-config`

### 4. Alter the config

- change `networking.hostId` in `hardware-configuration.nix` to match the output of `head -c 8 /etc/machine-id` on the target machine.  
  alternatively, this can be any random 8 character hex code.
- change `disk-config.nix` to match the desired disk layout. In particular, the `/dev/sd.*` references.
- in `hardware-configuration.nix`, rewrite the `fileSystem.*` entries to match the future disk layout. See other hosts for examples.

### 5. Run

First, run `git add hosts/<name>`. 

On your local machine, run:
```shell
cd init
nix-shell -p nixos-anywhere
# on an uninitialized host
nixos-anywhere --flake .#<name> root@<ip>
# to nuke an existing host
nixos-anywhere --flake .#<name> -p 666 -i <key file> cicd@<ip>
```

You should be able to SSH to the host now using
```shell
ssh -p666 -o "IdentitiesOnly=yes" -i ci_key_ed25519 cicd@<ip>
```
