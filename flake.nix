{
  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    nixos.url = "path:/home/jeff/dotfiles/nix";
  };

  outputs = { flake-utils, nixpkgs, nixos, ... }:
  flake-utils.lib.eachDefaultSystem (system:
    let
      hostname = "ceres";
      config = nixos.nixosConfigurations.${hostname}.config;

      pkgs = nixpkgs.legacyPackages.${system};
      kernel = config.boot.kernelPackages.kernel;
      kernelBuild = "${kernel.dev}/lib/modules/${kernel.modDirVersion}/build";
    in
    {
      devShells.default = pkgs.mkShell {
        name = "kernel-dev";

        packages = with pkgs; [
          gcc
          gnumake
          kernel.dev
          kmod
        ];

        CC = "gcc";
        KDIR = "${kernelBuild}";

        # Generates the clangd configuration file dynamically.
        #
        # This is required instead of keeping it source controlled because the
        # path that Nix gives to the kernel package is not consistent.
        #
        # TODO: Support x86 kernel development platforms.
        shellHook = ''
          cat > .clangd <<EOF
          CompileFlags:
            Add:
              - '-x'
              - 'c'
              - '-Wall'
              - '-nostdinc'
              - '-std=gnu11'
              - '-I${kernelBuild}/include'
              - '-I${kernelBuild}/source/include'
              - '-I${kernelBuild}/source/arch/arm64/include'
              - '-I${kernelBuild}/arch/arm64/include/generated'
              - '-I${kernelBuild}/arch/arm64/include/generated/uapi'
              - '-I${kernelBuild}/source/include/uapi'
              - '-I${kernelBuild}/include/generated/uapi'
              - '-include'
              - '${kernelBuild}/source/include/linux/compiler-version.h'
              - '-include'
              - '${kernelBuild}/source/include/linux/kconfig.h'
              - '-include'
              - '${kernelBuild}/source/include/linux/compiler_types.h'
              - '-DMODULE'
              - '-D__KERNEL__'
          EOF
        '';
      };
    }
  );
}
