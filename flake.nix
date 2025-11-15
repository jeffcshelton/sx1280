{
  description = "SX1280 RF Transceiver Linux Kernel Module";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.05";
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

        dtso = ''
          /dts-v1/;
          /plugin/;

          / {
            fragment@0 {
              target = <&spi0>;

              #address-cells = <1>;
              #size-cells = <0>;

              sx1280: sx@0 {
                compatible = "semtech,sx1280";
                reg = <0>;

                busy-gpios = <&gpio 17 0>;
                dio-gpios = <&gpio 22 0>, <0 0>, <0 0>;
              };
            };

            fragment@1 {
              target = <&spidev0>;
              __overlay__ {
                status = "disabled";
              };
            };
          };
        '';
      in
      {
        options.hardware.sx1280 = {
          enable = mkEnableOption "Enable the SX1280 Linux kernel driver";

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

          busyGPIO = mkOption {
            # TODO
          };

          frequency = mkOption {
            type = types.ints.u32;
            default = 2400000000;
          };

          startupTimeout = mkOption {
            type = types.ints.between 0 1000000;
            default = 2000;
          };

          txTimeout = mkOption {
            type = types.ints.between 0 262143999;
            default = 1000;
          };

          power = mkOption {
            type = types.ints.between (-18) 13;
            default = 13;
          };

          rampTime = mkOption {
            type = types.enum [ 2 4 6 8 10 12 16 20 ];
            default = 2;
          };

          ble = {
            maxPayloadBytes = mkOption {
              type = types.enum [ 31 37 63 255 ];
            };

            crcBytes = mkOption {
              type = types.enum [ 0 3 ];
            };

            testPayload = mkOption {
              type = types.enum [
                "prbs9"
                "eyelong10"
                "eyeshort10"
                "prbs15"
                "all1"
                "all0"
                "eyelong01"
                "eyeshort01"
              ];
            };

            disableWhitening = mkOption {
              type = types.bool;
              default = false;
            };

            accessAddress = mkOption {
              type = types.int;
            };

            crcSeed = mkOption {
              type = types.ints.u16;
            };
          };

          flrc = {
            bitrateKbs = mkOption {
              type = types.enum [ 1300 1000 650 520 325 260 ];
              default = 1300;
            };

            codingRate = mkOption {
              type = types.enum [ "1/2" "3/4" "1/1" ];
              default = "3/4";
            };

            bt = mkOption {
              type = types.enum [ "off" "1.0" "0.5" ];
              default = "1.0";
            };

            preambleBits = mkOption {
              type = types.enum [ 8 12 16 20 24 28 32 ];
              default = 8;
            };

            syncWordMatch = mkOption {
              # TODO : come back
            };

            fixedLength = mkOption {
              type = types.bool;
              default = false;
            };

            maxPayloadBytes = mkOption {
              type = types.ints.between 6 127;
              default = 127;
            };

            crcBytes = mkOption {
              type = types.enum [ 0 2 3 4 ];
              default = 2;
            };

            crcSeed = mkOption {
              type = types.ints.u16;
              default = 0;
            };

            disableWhitening = mkOption {
              type = types.bool;
              default = false;
            };

            syncWords = mkOption {
              # TODO : come back
            };
          };

          # TODO: come back and try to capture the more complicated valid pairs
          # between fields
          gfsk = {
            bitrateKbs = mkOption {
              type = types.enum [ 2000 1600 1000 800 500 400 250 125 ];
              default = 2000;
            };

            bandwidthKHz = mkOption {
              type = types.enum [ 2400 1200 600 300 ];
              default = 2400;
            };

            modulationIndex = mkOption {
              type = types.enum [
                35
                50
                75
                100
                125
                150
                175
                200
                225
                250
                275
                300
                325
                350
                375
                400
              ];

              default = 200;
            };

            bt = mkOption {
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

            syncWordMatch = mkOption {
              # TODO : come back
            };

            fixedLength = mkOption {
              type = types.bool;
              default = false;
            };

            maxPayloadBytes = mkOption {
              type = types.ints.between 1 255;
              default = 255;
            };

            crcBytes = mkOption {
              type = types.ints.between 0 2;
              default = 2;
            };

            disableWhitening = mkOption {
              type = types.bool;
              default = false;
            };

            syncWords = mkOption {
              # TODO
            };

            crcSeed = mkOption {
              type = types.ints.u16;
              default = 0;
            };

            crcPolynomial = {
              type = types.ints.u16;
              default = 0;
            };
          };

          lora = {
            spreadingFactor = mkOption {
              type = types.ints.between 5 12;
              default = 12;
              description = "spreading factor";
            };

            bandwidthKHz = mkOption {
              type = types.enum [ 1600 800 400 200 ];
              default = 1600;
              description = "The bandwidth";
            };

            codingRate = mkOption {
              type = types.enum [ "4/5" "4/6" "4/7" "4/8" ];
              default = "4/7";
              description = "";
            };

            disableLongInterleaving = mkOption {
              type = types.bool;
              default = false;
            };

            preambleBits = mkOption {
              type = types.ints.between 1 491520;
              default = 8;
            };

            implicitHeader = mkOption {
              type = types.bool;
              default = false;
            };

            maxPayloadBytes = mkOption {
              type = types.ints.between 1 255;
              default = 255;
            };

            disableCRC = mkOption {
              type = types.bool;
              default = false;
            };

            invertIQ = mkOption {
              type = types.bool;
              default = false;
            };
          };
        };

        config = lib.mkIf cfg.enable {
          boot = {
            extraModulePackages = [ kModule ];
            kernelModules = [ "sx1280" ];
          };

          # hardware.deviceTree = {
          #   enable = true;
          #   overlays = [
          #     {
          #       name = "sx1280.dtbo";
          #       dtsFile = cfg.dtso;
          #     }
          #   ];
          # };
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
          EOF
        '';
      };
    }
  );
}
