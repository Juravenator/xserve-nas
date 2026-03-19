# Running the setup from your local machine

> [!CAUTION]
> Do this only when you know what you're doing.
> Make sure you don't target production servers when you're not intending to.
> Comment out other hosts in [flake.nix] when you're unsure.

You will need passwordless sudo access to the host if you want to proceed.

```shell
nix run .#colmena -- apply --on xserve
```