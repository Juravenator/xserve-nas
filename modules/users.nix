{ config, lib, pkgs, modulesPath, ... }:let
  mkUser = import ./make-user.nix { inherit lib pkgs; };
in {
  programs.zsh = {
    enable = true;
    enableBashCompletion = true;
    autosuggestions.enable = true;
    syntaxHighlighting.enable = true;
  };
  users.defaultUserShell = pkgs.zsh;

  programs.zsh.ohMyZsh = {
    enable = true;
    plugins = [ "git" ];
    custom = "$HOME/.oh-my-zsh/custom/";
    # theme = "powerlevel10k/powerlevel10k";
    theme = "bureau";
  };

  environment.systemPackages = with pkgs; [
    lolcat
  ];

  services.openssh = {
    settings = {
      PrintMotd = false;
      PrintLastLog = false;
    };
  };

  # environment.etc."motd.d/banners".source = ./sshd-motd/banners;
  # environment.etc."update-motd.d/00-header" = {
  #   source = ./sshd-motd/00-header;
  #   mode = "0755";
  # };

  # programs.zsh.shellInit = ''
  #   if [[ $- == *i* ]]; then
  #     . /etc/update-motd.d/00-header
  #   fi
  # '';

  environment.etc."skel/.zshrc".text = ''
    # Lines configured by zsh-newuser-install
    HISTFILE=~/.zsh_history
    HISTSIZE=1000
    SAVEHIST=1000
    setopt autocd extendedglob nomatch notify
    bindkey -e
    # End of lines configured by zsh-newuser-install
  '';

  imports = [
    (mkUser "jura" "https://github.com/juravenator.keys")
  ];

}
