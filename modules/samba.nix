{ config, lib, pkgs, modulesPath, ... }:

{
  imports = [ ];
  
  users.groups.data.gid = 950;

  users.users.data = {
    uid = 3000;
    group = "data";
    isSystemUser = true;
  };

  services.samba = {
    enable = true;
    openFirewall = true;

    settings = {
      global = {
        workgroup = "WORKGROUP";
        security = "user";

        # Required for ACL support
        "vfs objects" = "acl_xattr";
        "map acl inherit" = "yes";
        "store dos attributes" = "yes";
      };

      hot-1 = {
        path = "/hot-1";
        browseable = "yes";
        "read only" = "no";
        "guest ok" = "no";

        "force user" = "data";
        "force group" = "data";

        "create mask" = "0660";
        "directory mask" = "0770";

        "inherit permissions" = "yes";
        "inherit acls" = "yes";
      };

      warm-1 = {
        path = "/warm-1";
        browseable = "yes";
        "read only" = "no";
        "guest ok" = "no";

        "force user" = "data";
        "force group" = "data";

        "create mask" = "0660";
        "directory mask" = "0770";

        "inherit permissions" = "yes";
        "inherit acls" = "yes";
      };
    };
  };
}