{ name, config, lib, pkgs, modulesPath, ... }:

let
  newDiskSleepService = {id, name, S}: {
    "disk-sleep-${name}" = {
      enable = true;
      description = "ensure correct idle spindown on ${name} (${id})";

      wantedBy = [ "sysinit.target" ];
      serviceConfig = {
        Type = "oneshot";
        RemainAfterExit = true;
        # A value of zero means "timeouts are disabled".
        # Values from 1 to 240 specify multiples of 5 seconds.
        # Values from 241 to 251 specify from 1 to 11 units of 30 minutes.
        # A value of 252 signifies a timeout of 21 minutes.
        # A value of 253 sets a vendor-defined timeout period between 8 and 12 hours.
        # The value 254 is  reserved.
        # 255 is interpreted as 21 minutes plus 15 seconds.
        ExecStart = "${pkgs.hdparm}/bin/hdparm -q -S ${S} /dev/disk/by-id/${id}";
      };
    };
  };
in {
  imports = [ ];

  environment.etc."start-storage.sh" = {
    source = ./zpools/start-storage.sh;
    mode = "0744";
  };


  environment.systemPackages = with pkgs; [
    hdparm
  ];

  systemd.services = lib.attrsets.mergeAttrsList [
    (newDiskSleepService {name = "cold-2-1"; id = "ata-ST4000DM004-2CV104_ZFN3BKLL"; S = "241";})
    (newDiskSleepService {name = "cold-2-2"; id = "ata-ST4000DM004-2CV104_ZTT0MQVY"; S = "241";})
    (newDiskSleepService {name = "cold-2-3"; id = "ata-ST4000DM004-2CV104_ZFN3AVWC"; S = "241";})
    (newDiskSleepService {name = "cold-2-4"; id = "ata-ST4000DM004-2CV104_ZFN3AW1F"; S = "241";})
    (newDiskSleepService {name = "cold-2-5"; id = "ata-ST4000DM004-2CV104_ZTT0MPWE"; S = "241";})

    (newDiskSleepService {name = "cold-1-1"; id = "ata-ST16000NM001G-2KK103_ZL2MD6ZC"; S = "241";})
    (newDiskSleepService {name = "cold-1-2"; id = "ata-ST16000NM001G-2KK103_ZL2MBJN6"; S = "241";})
    (newDiskSleepService {name = "cold-1-3"; id = "ata-ST16000NM001G-2KK103_ZL2KZ0DZ"; S = "241";})
    (newDiskSleepService {name = "cold-1-4"; id = "ata-ST16000NM001G-2KK103_ZL2MBJ78"; S = "241";})
    (newDiskSleepService {name = "cold-1-5"; id = "ata-ST16000NM001G-2KK103_ZL2L5LHT"; S = "241";})

    (newDiskSleepService {name = "warm-1-1"; id = "ata-ST16000NM001G-2KK103_ZL2MCW9Y"; S = "242";})
    (newDiskSleepService {name = "warm-1-2"; id = "ata-ST16000NM000J-2TW103_ZR5F455N"; S = "242";})
  ];
}