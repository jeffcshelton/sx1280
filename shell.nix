# Configures the shell for Linux kernel development.
#
# This dev shell is particularly unique because it tries to grab the kernel
# headers for the current kernel version, no matter what version the current
# machine is running.

let
  # At the moment, the system configuration path is assumed to be
  # ~/dotfiles/nix. Obviously, this does not work for all developers, as many do
  # not keep their configuration here. As a temporary fix, you can put a symlink
  # at ~/dotfiles/nix to your real configuration.
  #
  # Nix doesn't even guarantee that the configuration source files are actually
  # _on_ the machine, much less provide the user with the location of the
  # source. This presents significant with locating the configuration source
  # to determine the kernel package. The fact that Nix wants everything to be
  # perfectly reproducible makes this challenge even greater.
  #
  # TODO: Change this implementation to make it more robust for all developers.
  # The solution is likely to require developers to specify the path to their
  # configuration's source.
  arch = builtins.head (builtins.split "-" builtins.currentSystem);
  home = builtins.getEnv "HOME";
  host = builtins.getEnv "HOST";

  pkgs = import <nixpkgs> {};
  system = builtins.getFlake "path:${home}/dotfiles/nix";

  config = system.outputs.nixosConfigurations.${host}.config;
  kernel = config.boot.kernelPackages.kernel;
  kernelBuild = "${kernel.dev}/lib/modules/${kernel.modDirVersion}/build";

  # Determine the architecture string for arch-specific header includes.
  # These arch strings are slightly different than the output from
  # `currentSystem`.
  includeArch =
    if builtins.elem arch [ "x86" "x86_64" "amd64" ] then "x86"
    else if arch == "aarch64" then "arm64"
    else throw "Unsupported architecture: ${arch}";
in
pkgs.mkShell {
  buildInputs = with pkgs; [
    gcc
    gnumake
    kernel.dev
    kmod
  ];

  CC = "gcc";
  KDIR = "${kernelBuild}";

  # Generates the clangd configuration file dynamically.
  #
  # This is required instead of keeping it source controlled because the path
  # that Nix gives to the kernel package is not consistent.
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
    EOF
  '';
}
