{
  description = "SX1280 RF Transceiver Linux Kernel Module";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.11";
  };

  outputs = { self, flake-utils, nixpkgs }:
  let
    # Only aarch64-linux devices are supported by this driver.
    system = "aarch64-linux";
    pkgs = import nixpkgs { inherit system; };

    mkModule = kernel:
      let
        buildDir = "${kernel.dev}/lib/modules/${kernel.modDirVersion}/build";
        outDir = "$out/lib/modules/${kernel.modDirVersion}/kernel/drivers/net/wireless";
      in
      pkgs.stdenv.mkDerivation {
        pname = "sx1280-module";
        version = "1.0.0";
        src = ./.;

        nativeBuildInputs = kernel.moduleBuildDependencies;

        # Disable Nix hardening features that are incompatible with kernel
        # modules.
        hardeningDisable = [ "pic" "format" ];

        makeFlags = [
          "KERNELDIR=${buildDir}"
          "INSTALL_MOD_PATH=$(out)"
          "ARCH=${pkgs.stdenv.hostPlatform.linuxArch}"
        ];

        buildPhase = ''
          make -C ${buildDir} M=$(pwd) modules
        '';

        installPhase = ''
          mkdir -p ${outDir}
          cp sx1280.ko ${outDir}/
        '';

        meta = with pkgs.lib; {
          description = "SX1280 RF Transceiver Linux Kernel Driver";
          license = licenses.gpl2;
          platforms = platforms.linux;
        };
      };
  in
  {
    ${system}.mkModule = mkModule;

    nixosModules.default = { config, lib, pkgs, ... }:
      let
        inherit (lib) mkEnableOption mkOption types;

        cfg = config.hardware.sx1280;
        kernel = config.boot.kernelPackages.kernel;
        kModule = self.${pkgs.stdenv.hostPlatform.system}.mkModule kernel;
      in
      {
        options.hardware.sx1280 = {
          enable = mkEnableOption 
            "Whether to enable the SX1280 Linux kernel driver";

          dtso = mkOption {
            type = types.nullOr types.path;
            default = null;
          };

          mode = mkOption {
            type = types.enum [ "ble" "flrc" "gfsk" "lora" ];
            default = "gfsk";
            description = ''
              Operating mode of the transceiver.
              - ble: Bluetooth Low Energy
              - gfsk: Gaussian Frequency Shift Keying
              - flrc: Fast Long Range Communication
              - lora: LoRa modulation
            '';
          };

          frequencyMHz = mkOption {
            type = types.ints.between 2400 2500;
            default = 2400;
            description = "Frequency of the transmitter, in MHz.";
          };

          txPower = mkOption {
            type = types.ints.between (-18) 13;
            default = 13;
            description = "Transmitter power, in dBm.";
          };

          rampTimeUs = mkOption {
            type = types.enum [ 2 4 6 8 10 12 16 20 ];
            default = 2;
            example = 20;
            description = "Ramp time, in microseconds.";
          };

          syncWords = mkOption {
            type = types.addCheck
              (types.listOf
                (types.addCheck
                  types.str
                  (hex:
                    let i = lib.trivial.fromHexString hex;
                    in i >= 0 && i <= lib.trivial.fromHexString "FFFFFFFFFF"
                  )
                )
              )
              (arr: builtins.length arr <= 3);

            default = [ ];
            example = [ "12AD34CD56" "D391D391D3" "AAF0053C81" ];
          };

          crcSeed = mkOption {
            type = types.addCheck
              types.str
              (hex:
                let i = lib.trivial.fromHexString hex;
                in i >= 0 && i <= lib.trivial.fromHexString "FFFF"
              );

            default = "FFFF";
            example = "AAAA";
          };

          flrc = {
            bandwidthKHz = mkOption {
              type = types.enum [ 1200 600 300 ];
              default = 1200;
            };

            bitrateKbs = mkOption {
              type = types.enum [ 1300 1000 650 520 325 260 ];
              default = 1300;
            };

            codingRate = mkOption {
              type = types.enum [ "1/2" "3/4" "1/1" ];
              default = "3/4";
            };

            bandwidthTime = mkOption {
              type = types.enum [ "off" "1.0" "0.5" ];
              default = "1.0";
            };

            preambleBits = mkOption {
              type = types.enum [ 8 12 16 20 24 28 32 ];
              default = 8;
            };

            crcBytes = mkOption {
              type = types.enum [ 0 2 3 4 ];
              default = 2;
            };

            whitening = mkOption {
              type = types.bool;
              default = true;
            };
          };

          # TODO: come back and try to capture the more complicated valid pairs
          # between fields
          gfsk = {
            bandwidthKHz = mkOption {
              type = types.enum [ 2400 1200 600 300 ];
              default = 2400;
            };

            bitrateKbs = mkOption {
              type = types.enum [ 2000 1600 1000 800 500 400 250 125 ];
              default = 2000;
            };

            modulationIndex = mkOption {
              type = types.enum [
                "0.35"
                "0.50"
                "0.75"
                "1.00"
                "1.25"
                "1.50"
                "1.75"
                "2.00"
                "2.25"
                "2.50"
                "2.75"
                "3.00"
                "3.25"
                "3.50"
                "3.75"
                "4.00"
              ];

              default = "2.00";
            };

            bandwidthTime = mkOption {
              type = types.enum [ "off" "1.0" "0.5" ];
              default = "1.0";
            };

            preambleBits = mkOption {
              type = types.enum [ 4 8 12 16 20 24 28 32 ];
              default = 8;
            };

            syncWordBytes = mkOption {
              type = types.ints.between 1 5;
              default = 2;
            };

            crcBytes = mkOption {
              type = types.ints.between 0 2;
              default = 2;
            };

            whitening = mkOption {
              type = types.bool;
              default = true;
            };

            crcPolynomial = mkOption {
              type = types.addCheck
                types.str
                (hex:
                  let i = lib.trivial.fromHexString hex;
                  in i >= 0 && i <= lib.trivial.fromHexString "FFFF"
                );

              default = "1021";
              example = "8005";
            };

            syncWordMatch = mkOption {
              type = types.addCheck
                (types.listOf types.bool)
                (x: builtins.length x == 3);

              default = [ true false false ];
              example = [ true true true ];
            };
          };

          lora = {
            bandwidthKHz = mkOption {
              type = types.enum [ 1600 800 400 200 ];
              default = 1600;
            };

            codingRate = mkOption {
              type = types.enum [
                "4/5"
                "4/6"
                "4/7"
                "4/8"
                "4/5*"
                "4/6*"
                "4/8*"
              ];
              default = "4/7";
            };

            crcEnable = mkOption {
              type = types.bool;
              default = true;
            };

            invertIQ = mkOption {
              type = types.bool;
              default = false;
            };

            preambleBits = mkOption {
              type = types.ints.between 1 491520;
              default = 8;
            };

            spreadingFactor = mkOption {
              type = types.ints.between 5 12;
              default = 12;
              description = "spreading factor";
            };
          };
        };

        config = lib.mkIf cfg.enable {
          assertions = [
            /* GFSK birate-bandwidth pair validation. */
            {
              assertion =
                let
                  valid = [
                    { bitrateKbs = 2000; bandwidthKHz = 2400; }
                    { bitrateKbs = 1600; bandwidthKHz = 2400; }
                    { bitrateKbs = 1000; bandwidthKHz = 2400; }
                    { bitrateKbs = 1000; bandwidthKHz = 1200; }
                    { bitrateKbs = 800;  bandwidthKHz = 2400; }
                    { bitrateKbs = 800;  bandwidthKHz = 1200; }
                    { bitrateKbs = 500;  bandwidthKHz = 1200; }
                    { bitrateKbs = 500;  bandwidthKHz = 600;  }
                    { bitrateKbs = 400;  bandwidthKHz = 1200; }
                    { bitrateKbs = 400;  bandwidthKHz = 600;  }
                    { bitrateKbs = 250;  bandwidthKHz = 600;  }
                    { bitrateKbs = 250;  bandwidthKHz = 300;  }
                    { bitrateKbs = 125;  bandwidthKHz = 300;  }
                  ];
                in
                builtins.any (combo:
                  combo.bandwidthKHz == cfg.gfsk.bandwidthKHz
                  && combo.bitrateKbs == cfg.gfsk.bitrateKbs
                ) valid;
            }

            /* FLRC bitrate-bandwidth pair validation. */
            {
              assertion =
                let
                  valid = [
                    { bitrateKbs = 1300; bandwidthKHz = 1200; }
                    { bitrateKbs = 1000; bandwidthKHz = 1200; }
                    { bitrateKbs = 650;  bandwidthKHz = 600;  }
                    { bitrateKbs = 520;  bandwidthKHz = 600;  }
                    { bitrateKbs = 325;  bandwidthKHz = 300;  }
                    { bitrateKbs = 260;  bandwidthKHz = 300;  }
                  ];
                in
                builtins.any (combo:
                  combo.bandwidthKHz == cfg.flrc.bandwidthKHz
                  && combo.bitrateKbs == cfg.flrc.bitrateKbs
                ) valid;
            }
          ];

          boot = {
            extraModulePackages = [ kModule ];
            kernelModules = [ "sx1280" ];
          };

          hardware.deviceTree = lib.mkIf (cfg.dtso != null) {
            enable = true;
            overlays = [
              {
                name = "sx1280.dtbo";
                dtsFile = cfg.dtso;
              }
            ];
          };

          services.udev.extraRules = ''
            # Configure the SX1280 via sysfs when it appears.
            SUBSYSTEM=="net", DRIVER=="sx1280", ACTION=="add", \
              ATTR{frequency}="${toString (cfg.frequencyMHz * 1000000)}", \
              ATTR{mode}="${cfg.mode}", \
              ATTR{tx_power}="${toString (cfg.txPower)}", \
              ATTR{ramp_time}="${toString (cfg.rampTimeUs)}", \
              ATTR{crc_seed}="${cfg.crcSeed}", \
              ATTR{flrc/bandwidth_time}="${cfg.flrc.bandwidthTime}", \
              ATTR{flrc/bitrate_bandwidth}="${
                toString (cfg.flrc.bitrateKbs * 1000)
              },${
                toString (cfg.flrc.bandwidthKHz * 1000)
              }", \
              ATTR{flrc/coding_rate}="${cfg.flrc.codingRate}", \
              ATTR{flrc/crc_bytes}="${toString(cfg.flrc.crcBytes)}", \
              ATTR{flrc/preamble_bits}="${toString(cfg.flrc.preambleBits)}", \
              ATTR{flrc/whitening}="${
                if cfg.flrc.whitening then "1" else "0"
              }", \
              ATTR{gfsk/bandwidth_time}="${cfg.gfsk.bandwidthTime}", \
              ATTR{gfsk/bitrate_bandwidth}="${
                toString (cfg.gfsk.bitrateKbs * 1000)
              },${
                toString (cfg.gfsk.bandwidthKHz * 1000)
              }", \
              ATTR{gfsk/crc_length}="${toString (cfg.gfsk.crcBytes)}", \
              ATTR{gfsk/crc_polynomial}="${cfg.gfsk.crcPolynomial}", \
              ATTR{gfsk/modulation_index}="${cfg.gfsk.modulationIndex}", \
              ATTR{gfsk/preamble_bits}="${toString (cfg.gfsk.preambleBits)}", \
              ATTR{gfsk/sync_word_length}="${
                toString (cfg.gfsk.syncWordBytes)
              }", \
              ATTR{gfsk/sync_word_match}="${
                lib.concatMapStrings
                  (b: if b then "1" else "0")
                  cfg.gfsk.syncWordMatch
              }", \
              ATTR{gfsk/whitening}="${
                if cfg.gfsk.whitening then "1" else "0"
              }", \
              ATTR{lora/bandwidth}="${
                toString (cfg.lora.bandwidthKHz * 1000)
              }", \
              ATTR{lora/coding_rate}="${cfg.lora.codingRate}", \
              ATTR{lora/crc_enable}="${
                if cfg.lora.crcEnable then "1" else "0"
              }", \
              ATTR{lora/invert_iq}="${
                if cfg.lora.invertIQ then "1" else "0"
              }", \
              ATTR{lora/preamble_bits}="${toString (cfg.lora.preambleBits)}", \
              ATTR{lora/spreading_factor}="${
                toString (cfg.lora.spreadingFactor)
              }"
          '';
        };
      };

    packages.${system}.default = mkModule pkgs.linuxPackages.kernel;
  } // flake-utils.lib.eachDefaultSystem (system:
    let
      pkgs = import nixpkgs { inherit system; };
      kernel = pkgs.linuxPackages.kernel;
      kernelBuild = "${kernel.dev}/lib/modules/${kernel.modDirVersion}/build";

      arch = builtins.head (builtins.split "-" system);
      includeArch =
        if builtins.elem arch [ "x86" "x86_64" "amd64" ] then "x86"
        else if arch == "aarch64" then "arm64"
        else throw "Unsupported architecture: ${arch}";
    in
    {
      devShells.default = pkgs.mkShell {
        buildInputs = with pkgs; [
          gcc
          gnumake
          kernel.dev
        ];

        CC = "gcc";
        KERNELDIR = "${kernelBuild}";

        shellHook = ''
          cat > .clangd <<EOF
          CompileFlags:
            Add:
              - '-x'
              - 'c'
              - '-Wall'
              - '-nostdinc'
              - '-std=gnu11'
              - '-I${kernelBuild}/arch/${includeArch}/include/generated'
              - '-I${kernelBuild}/arch/${includeArch}/include/generated/uapi'
              - '-I${kernelBuild}/include'
              - '-I${kernelBuild}/include/generated/uapi'
              - '-I${kernelBuild}/source/arch/${includeArch}/include'
              - '-I${kernelBuild}/source/include'
              - '-I${kernelBuild}/source/include/uapi'
              - '-include'
              - '${kernelBuild}/source/include/linux/compiler-version.h'
              - '-include'
              - '${kernelBuild}/source/include/linux/compiler_types.h'
              - '-include'
              - '${kernelBuild}/source/include/linux/kconfig.h'
              - '-DMODULE'
              - '-D__KERNEL__'
              - '-DKBUILD_MODNAME'
              - '-DDEBUG'
          EOF
        '';
      };
    }
  );
}
