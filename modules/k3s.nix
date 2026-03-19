{ name, config, lib, pkgs, modulesPath, ... }:

let
  argocd-crds = pkgs.fetchFromGitHub {
    owner = "argoproj";
    repo = "argo-cd";
    rev = "v3.3.4";
    sha256 = "sha256-U4HTnzBzgCN06XNCNpIyG8lpTJT6l+l8Pkz08muwtzo=";
  };
in
{
  imports = [ ];

  environment.systemPackages = with pkgs; [
    kubernetes-helm
  ];

  services.k3s = {
    enable = true;
    role = "server";
    nodeName = name;

    # # $ zfs create -o mountpoint=/var/lib/rancher/k3s/agent/containerd/io.containerd.snapshotter.v1.zfs <zpool name>/containerd
    # extraFlags = [
    #   "--snapshotter=zfs"
    # ];

    disable = [
      "traefik" # we maintain our own version
    ];

    manifests = {
      crd-application.source = "${argocd-crds}/manifests/crds/application-crd.yaml";
      crd-applicationset.source = "${argocd-crds}/manifests/crds/applicationset-crd.yaml";
      crd-appproject.source = "${argocd-crds}/manifests/crds/appproject-crd.yaml";
    };

    autoDeployCharts.traefik2 = {
      repo = "https://traefik.github.io/charts";
      name = "traefik";
      version = "39.0.5";
      hash = "sha256-LWl7boE85UG4Is7POi/2/LlzImDS+z56lzc4iqOb8vU=";
      targetNamespace = "traefik";
      createNamespace = true;

      values = {
        additionalArguments = [
          "--entryPoints.websecure.http.tls.certResolver=letsencrypt"
          "--certificatesresolvers.letsencrypt.acme.caServer=https://acme-v02.api.letsencrypt.org/directory"
          "--certificatesresolvers.letsencrypt.acme.email=glenn.dirkx@gmail.com"
          "--certificatesresolvers.letsencrypt.acme.httpChallenge.entryPoint=web"
          "--certificatesresolvers.letsencrypt.acme.storage=/data/acme.json"
        ];
        ports = {
          web = {
            http = {
              redirections = {
                entryPoint = {
                  to = "websecure";
                  seche = "https";
                  permanent = false;
                };
              };
            };
          };
        };
        ingressRoute = {
          dashboard = {
            enabled = true;
            matchRule = "Host(`traefik.nest.jurabytes.net`)";
            entryPoints = ["websecure"];
            middlewares = [{
              name = "google-oauth";
              namespace = "traefik";
            }];
          };
        };
        persistence = {
          enabled = true;
          existingClaim = "traefik-data";
        };
        providers = {
          kubernetesCRD = {
            allowCrossNamespace = true;
          };
        };
        extraObjects = [
          {
            apiVersion = "traefik.io/v1alpha1";
            kind = "Middleware";
            metadata = {
              name = "internal-only";
              namespace = "traefik";
            };
            spec = {
              ipAllowList = {
                sourceRange = [
                  "192.168.42.0/24"
                ];
              };
            };
          }
          {
            apiVersion = "traefik.io/v1alpha1";
            kind = "Middleware";
            metadata = {
              name = "google-oauth";
              namespace = "traefik";
            };
            spec = {
              forwardAuth = {
                address = "http://google-oauth.traefik";
                trustForwardHeader = true;
                authResponseHeaders = [
                  "X-Forwarded-User"
                ];
              };
            };
          }
          {
            apiVersion = "v1";
            kind = "PersistentVolume";
            metadata = {
              name = "traefik-data";
            };
            spec = {
              capacity = {
                storage = "100Mi";
              };
              volumeMode = "Filesystem";
              accessModes = [
                "ReadWriteOnce"
              ];
              persistentVolumeReclaimPolicy = "Retain";
              storageClassName = "traefik-data";
              local = {
                path = "/hot-1/apps/traefik/data";
              };
              nodeAffinity = {
                required = {
                  nodeSelectorTerms = [
                    {
                      matchExpressions = [
                        {
                          key = "kubernetes.io/hostname";
                          operator = "In";
                          values = [
                            "xserve"
                          ];
                        }
                      ];
                    }
                  ];
                };
              };
            };
          }
          {
            apiVersion = "v1";
            kind = "PersistentVolumeClaim";
            metadata = {
              name = "traefik-data";
            };
            spec = {
              accessModes = [
                "ReadWriteOnce"
              ];
              resources = {
                requests = {
                  storage = "100Mi";
                };
              };
              storageClassName = "traefik-data";
              volumeMode = "Filesystem";
              volumeName = "traefik-data";
            };
          }
          {
            apiVersion = "apps/v1";
            kind = "Deployment";
            metadata = {
              name = "google-oauth";
              namespace = "traefik";
            };
            spec = {
              replicas = 1;
              selector = {
                matchLabels = {
                  app = "google-oauth";
                };
              };
              template = {
                metadata = {
                  labels = {
                    app = "google-oauth";
                  };
                };
                spec = {
                  containers = [{
                    name = "traefik-forward-auth";
                    image = "thomseddon/traefik-forward-auth:2";
                    env = [
                      # These depend on `traefik-secret.yaml`
                      # being manually applied to the cluster.
                      {
                        name = "PROVIDERS_GOOGLE_CLIENT_ID";
                        valueFrom = {
                          secretKeyRef = {
                            name = "auth";
                            key = "clientID";
                          };
                        };
                      }
                      {
                        name = "PROVIDERS_GOOGLE_CLIENT_SECRET";
                        valueFrom = {
                          secretKeyRef = {
                            name = "auth";
                            key = "clientSecret";
                          };
                        };
                      }
                      {
                        name = "SECRET";
                        valueFrom = {
                          secretKeyRef = {
                            name = "auth";
                            key = "cookieSecret";
                          };
                        };
                      }
                      {
                        name = "INSECURE_COOKIE";
                        value = "true";
                      }
                    ];
                  }];
                };
              };
            };
          }
          {
            apiVersion = "v1";
            kind = "Service";
            metadata = {
              name = "google-oauth";
              namespace = "traefik";
            };
            spec = {
              ports = [{
                name = "http";
                targetPort = 4181;
                port = 80;
              }];
              selector = {
                app = "google-oauth";
              };
            };
          }
        ];
      };
    };

    autoDeployCharts.argocd = {
      repo = "https://argoproj.github.io/argo-helm";
      name = "argo-cd";
      version = "9.4.14";
      hash = "sha256-gnW01KZp9k0wJvoA6aDEf6v4kbXpgKWeeGuqGCxo3+w=";
      targetNamespace = "argocd";
      createNamespace = true;
      values = {
        global = {
          domain = "argocd.nest.jurabytes.net";
        };
        crds = {
          install = false;
        };
        configs = {
          params = {
            "server.insecure" = "true";
          };
          cm = {
            url = "https://argocd.nest.jurabytes.net";
            "admin.enabled" = false;
            "dex.config" = ''
              connectors:
              - config:
                  issuer: https://accounts.google.com
                  # These depend on `argocd-secret.yaml`
                  # being manually applied to the cluster.
                  clientID: $oidc.google.clientID
                  clientSecret: $oidc.google.clientSecret
                  insecureSkipVerify: true
                  userIDKey: email
                  userNameKey: email
                type: oidc
                id: google
                name: Google
              '';
            # Backup config for use in case of Dex troubles.
            # "oidc.config" = ''
            #   name: Google
            #   issuer: https://accounts.google.com
            #   clientID: $oidc.google.clientID
            #   clientSecret: $oidc.google.clientSecret
            #   requestedScopes: ["openid", "profile", "email"]
            #   '';

          };
          rbac = {
            "policy.csv" = ''
              p, role:operator, applications, sync, *, allow
              p, role:operator, applications, get, *, allow
              p, role:operator, applicationsets, get, *, allow
              p, role:operator, projects, get, *, allow
              p, role:operator, clusters, get, *, allow
              p, role:operator, repositories, get, *, allow
              p, role:operator, logs, get, *, allow

              g, glenn.dirkx@gmail.com, role:admin
              '';
          };
          secret = {
            createSecret = false;
          };
        };
        server = {
          ingress = {
            enabled = true;
          };
        };
        extraObjects = [
          {
            apiVersion = "argoproj.io/v1alpha1";
            kind = "Application";
            metadata = {
              name = "infra";
              namespace = "argocd";
            };
            spec = {
              project = "default";

              source = {
                repoURL = "https://github.com/juravenator/xserve-nas.git";
                targetRevision = "stable";
                path = "argocd";
              };

              destination = {
                server = "https://kubernetes.default.svc";
                namespace = "argocd";
              };

              syncPolicy = {
                automated = {
                  prune = true;
                  selfHeal = true;
                };
              };
            };
          }
        ];
      };
    };

    autoDeployCharts.openebs = {
      repo = "https://openebs.github.io/openebs";
      name = "openebs";
      version = "4.4.0";
      hash = "sha256-mrxD80vqkPh2NcBzDYz/b0I1WUp2GJirBmbdgSQB5cg=";
      targetNamespace = "openebs";
      createNamespace = true;
      values = {
        engines = {
          replicated = {
            mayastor = {
              enabled = false;
            };
          };
          local = {
            lvm = {
              enabled = false;
            };
          };
        };
        loki = {
          enabled = false;
        };
        minio = {
          enabled = false;
        };
        alloy = {
          enabled = false;
        };
        # localpv-provisioner = {
        #   # localpv = {
        #   #   enabled = false;
        #   # };
        # };
        zfs-localpv = {
          # crds = {
          #   csi = {
          #     volumeSnapshots = {
          #       enabled = true;
          #     }
          #   };
          # };
          zfs = {
            bin = "/run/current-system/sw/bin/zfs";
            # bin = "${pkgs.zfs}/bin/zfs";
          };
        };
        # lvm-localpv = {
        #   crds = {
        #     enabled = false;
        #   }
        # };
      };
    };
  };
}
