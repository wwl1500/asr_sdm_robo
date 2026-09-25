#!/usr/bin/env bash
# Set up development environment for asr_sdm_robo.
# Usage: ./setup-dev-env.sh [OPTIONS]
# Note: -y option is only for CI.

set -euo pipefail

option_common=false
option_dependency=false
option_ros=false
option_nvidia=false
option_no_nvidia=false
option_no_cuda_drivers=false
option_runtime=false
option_yes=false
option_list=false

# The only ROS 2 distribution supported on Ubuntu 24.04 by this workspace
ros_distro="jazzy"
# Keeps DDS traffic of this workspace separate from other ROS 2 machines on the LAN
ros_domain_id="100"
# Used when the ros-apt-source release cannot be queried from GitHub
ros_apt_source_fallback_version="1.2.0"

# Versions of the NVIDIA stack (see src/asr_sdm_universe/common/tensorrt_common/README.md)
nvidia_driver_branch="595"
cuda_version="13.2"
cuda_apt_suffix="13-2"          # apt package suffix for CUDA 13.2
cudnn_cuda_major="13"           # cuDNN 9 flavour matching the CUDA major version
tensorrt_version="11.1.0"
opencv_version="4.14.0"

# Workspace-local third-party sources. COLCON_IGNORE in dependency/ keeps colcon
# from treating them as packages when building the ROS 2 workspace.
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
dependency_dir="${script_dir}/dependency"
periphery_repo_url="https://github.com/AmpRobo/c-periphery.git"
periphery_src_dir="${dependency_dir}/c-periphery"
periphery_version="2.5.0"
periphery_install_prefix="/usr/local"
opencv_src_dir="${dependency_dir}/opencv-${opencv_version}"
opencv_contrib_src_dir="${dependency_dir}/opencv_contrib-${opencv_version}"

# CUDA library search paths, filled in by install_opencv_cuda
cuda_lib_dirs=()

# Env exports written to ~/.bashrc at the end of the NVIDIA role
nvidia_env_content=""

print_help() {
    echo "Usage: setup-dev-env.sh [OPTIONS]"
    echo "Options:"
    echo "  --help                  Display this help message"
    echo "  -h                      Display this help message"
    echo "  --list                  List all installable components"
    echo "  --common                Install common development packages (toolchain, git, python3, CLI tools, Coral GUI)"
    echo "  --ros                   Install ROS 2 ${ros_distro} desktop + rosdep/colcon (Ubuntu 24.04)"
    echo "  --dependency            Install third-party libraries (Pinocchio from robotpkg,"
    echo "                          c-periphery ${periphery_version} static library)"
    echo "  --nvidia                Install the NVIDIA stack (driver ${nvidia_driver_branch}, CUDA ${cuda_version},"
    echo "                          cuBLAS, NPP, cuDNN, TensorRT ${tensorrt_version}, OpenCV ${opencv_version} + CUDA)"
    echo "  -y                      Use non-interactive mode"
    echo "  -v                      Enable debug outputs"
    echo "  --no-nvidia             Disable installation of the NVIDIA-related roles ('cuda' and 'tensorrt')"
    echo "  --no-cuda-drivers       Disable installation of 'cuda-drivers' in the role 'cuda'"
    echo "  --runtime               Disable installation dev package of role 'cuda' and 'tensorrt'"
    echo "  --data-dir              Set data directory (default: $HOME/asr_sdm_data)"
    echo "  --download-artifacts    Download artifacts"
    echo "  --module                Specify the module (default: all)"
    echo "  --ros-distro            ROS distribution to install (only '${ros_distro}' is supported)"
    echo ""
}

print_list() {
    echo "Installable components:"
    echo ""
    echo "  Flag          Component    Description"
    echo "  ----          ---------    -----------"
    echo "  --common      Common       Install build toolchain, git, python3, CLI tools and Coral GUI libraries"
    echo "  --ros         ROS 2        Install ROS 2 ${ros_distro} desktop + rosdep/colcon (Ubuntu 24.04),"
    echo "                             and set ROS_DOMAIN_ID=${ros_domain_id} in ~/.bashrc"
    echo "  --dependency  Dependency   Install the third-party libraries below"
    echo "  --nvidia      NVIDIA       Install the GPU stack below (Ubuntu 24.04, x86_64)"
    echo ""
    echo "  Components of --dependency:"
    echo "    Pinocchio       robotpkg apt (C++ library and Python bindings)"
    echo "    c-periphery     ${periphery_version} static library from dependency/c-periphery"
    echo "                    into ${periphery_install_prefix}"
    echo ""
    echo "  Components of --nvidia:"
    echo "    NVIDIA driver   ${nvidia_driver_branch} (cuda-drivers-${nvidia_driver_branch}; skip with --no-cuda-drivers)"
    echo "    CUDA Toolkit    ${cuda_version} (cuda-toolkit-${cuda_apt_suffix})"
    echo "    CUDA cuBLAS     libcublas-${cuda_apt_suffix}"
    echo "    CUDA NPP        libnpp-${cuda_apt_suffix} (plus cuFFT, required by OpenCV CUDA)"
    echo "    cuDNN           libcudnn9-cuda-${cudnn_cuda_major}"
    echo "    TensorRT        ${tensorrt_version} (needs the NVIDIA TensorRT local apt repo)"
    echo "    OpenCV + CUDA   ${opencv_version} into ~/.local/opencv-cuda"
    echo "                    (source in dependency/opencv-${opencv_version} and"
    echo "                     dependency/opencv_contrib-${opencv_version})"
    echo ""
    echo "Examples:"
    echo "  ./setup-dev-env.sh --list"
    echo "  ./setup-dev-env.sh --common"
    echo "  ./setup-dev-env.sh --ros"
    echo "  ./setup-dev-env.sh --dependency"
    echo "  ./setup-dev-env.sh --nvidia"
    echo "  ./setup-dev-env.sh --nvidia --runtime --no-cuda-drivers"
    echo "  ./setup-dev-env.sh --ros --nvidia --dependency -y"
}

run_cmd() {
    echo "Running: $*"
    "$@"
}

confirm_or_exit() {
    if [[ "${option_yes}" == true ]]; then
        export DEBIAN_FRONTEND=noninteractive
        return
    fi

    echo -e "\e[33mThis will install the selected development packages.\e[m"
    read -rp "> Continue? [y/N] " answer
    if ! [[ ${answer:0:1} =~ y|Y ]]; then
        echo -e "\e[33mCancelled.\e[0m"
        exit 1
    fi
}

apt_install() {
    if [[ "${option_yes}" == true ]]; then
        sudo apt-get --yes "$@"
    else
        sudo apt-get "$@"
    fi
}

install_common() {
    local -a common_packages

    if ! command -v apt-get >/dev/null 2>&1; then
        echo -e "\e[33mSkipping common packages: apt-get is not available.\e[m"
        return 0
    fi

    common_packages=(
        # Toolchain and build systems
        build-essential
        cmake
        ninja-build
        pkg-config
        ccache
        gdb
        clang-format
        # Version control
        git
        git-lfs
        # Fetching and archives
        ca-certificates
        curl
        wget
        gnupg
        unzip
        zip
        # Python
        python3
        python3-pip
        python3-venv
        python3-dev
        # Qt 6 and the QML modules needed at runtime by Qt Quick UIs
        qt6-base-dev
        qt6-declarative-dev
        qt6-tools-dev
        qml6-module-qtqml
        qml6-module-qtqml-workerscript
        qml6-module-qtquick
        qml6-module-qtquick-controls
        qml6-module-qtquick-layouts
        qml6-module-qtquick-templates
        qml6-module-qtquick-window
        # Coral GUI (OpenSceneGraph / osgOcean and VulkanSceneGraph)
        libopenscenegraph-dev
        libfftw3-dev
        libvulkan-dev
        libassimp-dev
        glslang-dev
        glslang-tools
        libxcb1-dev
        # Point Cloud Library and optional io pcap support
        libpcl-dev
        libpcap-dev
        # Everyday CLI tools
        vim
        tmux
        htop
        tree
        jq
        ripgrep
        # Distro and hardware inspection
        lsb-release
        software-properties-common
        net-tools
        iputils-ping
        pciutils
        usbutils
    )

    echo -e "\e[36mInstalling common development packages...\e[m"

    if ! command -v sudo >/dev/null 2>&1; then
        apt-get -y update
        apt-get -y install sudo
    fi

    apt_install update
    apt_install install "${common_packages[@]}"

    echo -e "\e[32mDone.\e[0m Common packages installed."
}

ensure_dependency_dir() {
    # Keep a COLCON_IGNORE marker so `colcon build` from the workspace root does not
    # treat third-party trees under dependency/ as packages.
    mkdir -p "${dependency_dir}"
    if [[ ! -f "${dependency_dir}/COLCON_IGNORE" ]]; then
        touch "${dependency_dir}/COLCON_IGNORE"
    fi
}

# Download a GitHub tag zip into dependency/<dir_name> when that tree is missing.
# GitHub extracts the archive as <repo>-<tag>/, which must match dir_name.
ensure_github_tag_source() {
    local repo="$1"
    local tag="$2"
    local dir_name="$3"
    local dest_dir="${dependency_dir}/${dir_name}"
    local zip_path="${dependency_dir}/${dir_name}.zip"
    local url="https://github.com/${repo}/archive/refs/tags/${tag}.zip"

    ensure_dependency_dir

    if [[ -f "${dest_dir}/CMakeLists.txt" ]]; then
        echo "${dir_name} source already present in ${dest_dir}, skipping download."
        return 0
    fi

    if [[ -e "${dest_dir}" ]]; then
        echo -e "\e[31m${dest_dir} exists but has no CMakeLists.txt; not overwriting it.\e[m" >&2
        return 1
    fi

    if [[ ! -f "${zip_path}" ]]; then
        echo -e "\e[36mDownloading ${repo} ${tag} into ${dependency_dir}...\e[m"
        run_cmd wget -q --show-progress -O "${zip_path}" "${url}"
    else
        echo "Using existing archive ${zip_path}."
    fi

    echo "Extracting ${zip_path}..."
    run_cmd unzip -q -d "${dependency_dir}" "${zip_path}"

    if [[ ! -f "${dest_dir}/CMakeLists.txt" ]]; then
        echo -e "\e[31mFailed to extract ${repo} ${tag} into ${dest_dir}.\e[m" >&2
        return 1
    fi
}

ensure_periphery_source() {
    ensure_dependency_dir

    if [[ -f "${periphery_src_dir}/CMakeLists.txt" ]]; then
        echo "c-periphery source already present in ${periphery_src_dir}, skipping clone."
        return 0
    fi

    if [[ -e "${periphery_src_dir}" ]]; then
        echo -e "\e[31m${periphery_src_dir} exists but has no CMakeLists.txt; not overwriting it.\e[m" >&2
        return 1
    fi

    echo -e "\e[36mCloning c-periphery into ${periphery_src_dir}...\e[m"
    run_cmd git clone --depth 1 "${periphery_repo_url}" "${periphery_src_dir}"
}

ensure_opencv_source() {
    ensure_github_tag_source "opencv/opencv" "${opencv_version}" "opencv-${opencv_version}"
    ensure_github_tag_source "opencv/opencv_contrib" "${opencv_version}" "opencv_contrib-${opencv_version}"
}

install_periphery() {
    local build_dir="${periphery_src_dir}/build"
    local static_lib=""
    local candidate
    local multiarch=""
    local -a generator

    echo -e "\e[36mInstalling c-periphery ${periphery_version} as a static library...\e[m"

    if ! command -v sudo >/dev/null 2>&1; then
        apt-get -y update
        apt-get -y install sudo
    fi

    apt_install update
    apt_install install \
        build-essential \
        cmake \
        git \
        pkg-config \
        ninja-build \
        linux-libc-dev

    ensure_periphery_source

    rm -rf "${build_dir}"
    mkdir -p "${build_dir}"

    if command -v ninja >/dev/null 2>&1; then
        generator=(-G Ninja)
    else
        generator=()
    fi

    echo "  SOURCE_DIR      = ${periphery_src_dir}"
    echo "  BUILD_DIR       = ${build_dir}"
    echo "  INSTALL_PREFIX  = ${periphery_install_prefix}"

    # CMAKE_INSTALL_LIBDIR=lib keeps libperiphery.a in /usr/local/lib so the
    # linker and find_package(periphery) both find it without a multiarch path.
    run_cmd cmake -S "${periphery_src_dir}" -B "${build_dir}" "${generator[@]}" \
        -D CMAKE_BUILD_TYPE=Release \
        -D CMAKE_INSTALL_PREFIX="${periphery_install_prefix}" \
        -D CMAKE_INSTALL_LIBDIR=lib \
        -D BUILD_SHARED_LIBS=OFF \
        -D BUILD_TESTS=OFF

    run_cmd cmake --build "${build_dir}" --parallel "$(nproc)"
    run_cmd sudo cmake --install "${build_dir}"

    if command -v dpkg-architecture >/dev/null 2>&1; then
        multiarch="$(dpkg-architecture -qDEB_HOST_MULTIARCH 2>/dev/null || true)"
    fi

    for candidate in \
        "${periphery_install_prefix}/lib/libperiphery.a" \
        "${periphery_install_prefix}/lib64/libperiphery.a" \
        "${periphery_install_prefix}/lib/${multiarch}/libperiphery.a"
    do
        if [[ -n "${candidate}" && -f "${candidate}" ]]; then
            static_lib="${candidate}"
            break
        fi
    done

    if [[ -z "${static_lib}" ]]; then
        echo -e "\e[31mFailed to install c-periphery: libperiphery.a was not found under ${periphery_install_prefix}.\e[m" >&2
        return 1
    fi

    echo -e "\e[32mDone.\e[0m c-periphery static library installed:"
    echo "  ${static_lib}"
    echo "CMake package: find_package(periphery)  (prefix ${periphery_install_prefix})"
}

ubuntu_codename() {
    # /etc/os-release works before lsb-release is installed, lsb_release is the fallback
    local codename=""

    if [[ -r /etc/os-release ]]; then
        codename="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}")"
    fi

    if [[ -z "${codename}" ]] && command -v lsb_release >/dev/null 2>&1; then
        codename="$(lsb_release -sc 2>/dev/null || true)"
    fi

    printf '%s' "${codename:-unknown}"
}

run_as_invoking_user() {
    # rosdep refuses to run as root, so drop back to the user behind sudo
    if [[ "${EUID}" -eq 0 && -n "${SUDO_USER:-}" && "${SUDO_USER}" != "root" ]]; then
        sudo -u "${SUDO_USER}" -H "$@"
    else
        "$@"
    fi
}

enable_ubuntu_universe() {
    # Matches both the classic sources.list format and the deb822 format used since 24.04
    if grep -Rqs -E '^(deb .*[[:space:]]universe([[:space:]]|$)|Components:.*[[:space:]]universe([[:space:]]|$))' \
        /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null; then
        echo "Ubuntu 'universe' component already enabled, skipping."
        return 0
    fi

    apt_install install software-properties-common
    run_cmd sudo add-apt-repository -y universe
}

configure_utf8_locale() {
    if [[ "$(locale charmap 2>/dev/null || true)" == "UTF-8" ]]; then
        export LANG="${LANG:-en_US.UTF-8}"
        return 0
    fi

    apt_install install locales
    run_cmd sudo locale-gen en_US en_US.UTF-8
    run_cmd sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    export LC_ALL=en_US.UTF-8
}

setup_ros_apt_repo() {
    local codename="$1"
    local source_version=""
    local deb_url
    local tmp_deb

    if [[ "$(dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null || true)" == "install ok installed" ]]; then
        echo "ros2-apt-source already installed, skipping."
        apt_install update
        return 0
    fi

    echo -e "\e[36mAdding the ROS 2 apt repository...\e[m"
    apt_install install ca-certificates curl gnupg

    source_version="$(curl -fsSL --retry 3 --retry-delay 2 \
        https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest 2>/dev/null |
        awk -F'"' '/"tag_name"/ { print $4; exit }' || true)"

    if [[ -z "${source_version}" ]]; then
        source_version="${ros_apt_source_fallback_version}"
        echo -e "\e[33mCould not query the latest ros-apt-source release; falling back to ${source_version}.\e[m"
    fi

    deb_url="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${source_version}/ros2-apt-source_${source_version}.${codename}_all.deb"
    tmp_deb="$(mktemp -t ros2-apt-source.XXXXXXXX.deb)"

    if ! run_cmd curl -fsSL --retry 3 --retry-delay 2 -o "${tmp_deb}" "${deb_url}"; then
        rm -f "${tmp_deb}"
        echo -e "\e[31mFailed to download ${deb_url}\e[m" >&2
        return 1
    fi

    if ! sudo dpkg -i "${tmp_deb}"; then
        apt_install install -f
    fi
    rm -f "${tmp_deb}"

    apt_install update
}

setup_rosdep() {
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        run_cmd sudo rosdep init
    else
        echo "rosdep already initialized, skipping 'rosdep init'."
    fi

    if ! run_as_invoking_user rosdep update --rosdistro "${ros_distro}"; then
        echo -e "\e[33m'rosdep update' failed (network?); re-run it manually before building.\e[m"
    fi
}

remove_legacy_ros_bashrc_lines() {
    # Drops the plain ROS block written by earlier versions of this script so that
    # write_bashrc_block stays the single source of truth
    local bashrc_path="${HOME}/.bashrc"
    local tmp_file

    [[ -f "${bashrc_path}" ]] || return 0
    grep -qE '^# ROS 2 [a-z]+ environment$' "${bashrc_path}" || return 0

    tmp_file="$(mktemp)"
    awk '
        /^# ROS 2 [a-z]+ environment$/ { skipping = 1; next }
        skipping == 1 && /^(source \/opt\/ros\/|source \/usr\/share\/colcon_cd\/|export _colcon_cd_root=)/ { next }
        { skipping = 0; print }
    ' "${bashrc_path}" > "${tmp_file}"
    cat "${tmp_file}" > "${bashrc_path}"
    rm -f "${tmp_file}"
    echo "Removed the legacy ROS 2 environment lines from ~/.bashrc."
}

install_ros() {
    local codename

    codename="$(ubuntu_codename)"
    if [[ "${codename}" != "noble" ]]; then
        echo -e "\e[33mSkipping ROS 2: unsupported Ubuntu (${codename}). Supported: noble (24.04).\e[m"
        return 0
    fi

    echo -e "\e[36mInstalling ROS 2 ${ros_distro}...\e[m"

    if ! command -v sudo >/dev/null 2>&1; then
        apt-get -y update
        apt-get -y install sudo
    fi

    apt_install update
    apt_install install \
        ca-certificates \
        curl \
        gnupg \
        git \
        build-essential \
        cmake \
        pkg-config \
        python3 \
        python3-pip \
        python3-venv

    configure_utf8_locale
    enable_ubuntu_universe

    if ! setup_ros_apt_repo "${codename}"; then
        echo -e "\e[31mSkipping ROS 2: the apt repository could not be set up.\e[m" >&2
        return 1
    fi

    if ! apt-cache show "ros-${ros_distro}-desktop" >/dev/null 2>&1; then
        echo -e "\e[33mSkipping ROS 2: 'ros-${ros_distro}-desktop' is not available for ${codename}.\e[m"
        return 0
    fi

    # ROS packages are built against the current Ubuntu updates, so refresh them first
    apt_install upgrade
    apt_install install \
        "ros-${ros_distro}-desktop" \
        ros-dev-tools \
        python3-rosdep \
        python3-colcon-common-extensions

    setup_rosdep

    remove_legacy_ros_bashrc_lines
    write_bashrc_block "asr_sdm ros2 env" "source /opt/ros/${ros_distro}/setup.bash
export ROS_DOMAIN_ID=${ros_domain_id}
if [ -f /usr/share/colcon_cd/function/colcon_cd.sh ]; then
    . /usr/share/colcon_cd/function/colcon_cd.sh
    export _colcon_cd_root=/opt/ros/${ros_distro}/
fi
if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
    . /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
fi"

    echo -e "\e[32mDone.\e[0m ROS 2 ${ros_distro} installed (ROS_DOMAIN_ID=${ros_domain_id})."
    echo "Reload: source ~/.bashrc  |  Verify: ros2 run demo_nodes_cpp talker"
}

install_dependency() {
    echo -e "\e[36mInstalling third-party libraries (c-periphery, Pinocchio)...\e[m"
    install_periphery
    install_pinocchio_from_apt
    echo -e "\e[32mDependency setup complete.\e[0m"
}

install_pinocchio_from_apt() {
    local robotpkg_key_url="http://robotpkg.openrobots.org/packages/debian/robotpkg.asc"
    local robotpkg_key_path="/etc/apt/keyrings/robotpkg.asc"
    local robotpkg_list_path="/etc/apt/sources.list.d/robotpkg.list"
    local bashrc_path="${HOME}/.bashrc"
    local python_version
    local python_version_short
    local codename
    local repo_line
    local -a pinocchio_packages

    python_version="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
    python_version_short="$(python3 -c 'import sys; print(f"{sys.version_info.major}{sys.version_info.minor}")')"

    pinocchio_packages=(
        "robotpkg-pinocchio"
        "robotpkg-py${python_version_short}-pinocchio"
        "robotpkg-coal"
        "robotpkg-py${python_version_short}-coal"
        "robotpkg-py${python_version_short}-eigenpy"
    )

    run_cmd sudo apt update
    run_cmd sudo apt install -qqy lsb-release curl

    run_cmd sudo mkdir -p /etc/apt/keyrings
    run_cmd sudo curl -fsSL "${robotpkg_key_url}" -o "${robotpkg_key_path}"

    codename="$(lsb_release -cs)"
    repo_line="deb [arch=amd64 signed-by=${robotpkg_key_path}] http://robotpkg.openrobots.org/packages/debian/pub ${codename} robotpkg"

    if [[ -f "${robotpkg_list_path}" ]] && grep -qxF "${repo_line}" "${robotpkg_list_path}"; then
        echo "robotpkg apt source already exists, skipping."
    else
        echo "Running: sudo tee -a ${robotpkg_list_path}"
        echo "${repo_line}" | sudo tee -a "${robotpkg_list_path}" >/dev/null
    fi

    run_cmd sudo apt update
    run_cmd sudo apt install --reinstall -qqy "${pinocchio_packages[@]}"

    if [[ -f "${bashrc_path}" ]] && grep -q "pinocchio robotpkg setup" "${bashrc_path}"; then
        echo "~/.bashrc already contains Pinocchio environment variables, skipping."
    else
        cat >> "${bashrc_path}" <<EOF

# >>> pinocchio robotpkg setup >>>
export PATH=/opt/openrobots/bin:\$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:\$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:\$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python${python_version}/site-packages:\$PYTHONPATH
export CMAKE_PREFIX_PATH=/opt/openrobots:\$CMAKE_PREFIX_PATH
# <<< pinocchio robotpkg setup <<<
EOF
        echo "Appended Pinocchio environment variables to ~/.bashrc. Reload or reopen the terminal."
    fi

    echo "Pinocchio installation and environment setup complete."
}

portable_home_path() {
    # Keep a leading home directory as a literal ${HOME} so generated env exports stay user-agnostic
    local path="$1"

    if [[ "${path}" == "${HOME}" ]]; then
        printf '${HOME}'
    elif [[ "${path}" == "${HOME}/"* ]]; then
        printf '${HOME}/%s' "${path#"${HOME}/"}"
    else
        printf '%s' "${path}"
    fi
}

portable_home_path_list() {
    # portable_home_path applied to every entry of a colon-separated path list
    local list="$1"
    local result=""
    local item

    local IFS=':'
    for item in ${list}; do
        [[ -n "${item}" ]] || continue
        if [[ -z "${result}" ]]; then
            result="$(portable_home_path "${item}")"
        else
            result="${result}:$(portable_home_path "${item}")"
        fi
    done

    printf '%s' "${result}"
}

write_bashrc_block() {
    # write_bashrc_block <marker> <content>; rewrites the block when it already exists
    local marker="$1"
    local content="$2"
    local bashrc_path="${HOME}/.bashrc"
    local begin_line="# >>> ${marker} >>>"
    local end_line="# <<< ${marker} <<<"
    local tmp_file

    touch "${bashrc_path}"

    if grep -qxF "${begin_line}" "${bashrc_path}"; then
        tmp_file="$(mktemp)"
        awk -v begin_line="${begin_line}" -v end_line="${end_line}" '
            $0 == begin_line { inside = 1 }
            inside == 0      { print }
            $0 == end_line   { inside = 0 }
        ' "${bashrc_path}" > "${tmp_file}"
        # Command substitution drops the trailing blank lines left behind by the removed block
        printf '%s\n' "$(cat "${tmp_file}")" > "${bashrc_path}"
        rm -f "${tmp_file}"
        echo "Refreshed '${marker}' in ~/.bashrc."
    else
        echo "Appended '${marker}' to ~/.bashrc."
    fi

    {
        echo ""
        echo "${begin_line}"
        echo "${content}"
        echo "${end_line}"
    } >> "${bashrc_path}"
}

apt_candidate_version() {
    # apt_candidate_version <package> <version-prefix>
    local pkg="$1"
    local prefix="$2"
    apt-cache madison "${pkg}" 2>/dev/null | awk -F'|' -v p="${prefix}" '
        { gsub(/^[ \t]+|[ \t]+$/, "", $2); if (index($2, p) == 1) { print $2; exit } }'
}

check_nvidia_prerequisites() {
    local codename
    local arch

    codename="$(ubuntu_codename)"
    arch="$(dpkg --print-architecture 2>/dev/null || echo unknown)"

    if [[ "${codename}" != "noble" ]]; then
        echo -e "\e[33mSkipping NVIDIA stack: unsupported Ubuntu (${codename}). Supported: noble (24.04).\e[m"
        return 1
    fi

    if [[ "${arch}" != "amd64" ]]; then
        echo -e "\e[33mSkipping NVIDIA stack: unsupported architecture (${arch}). Supported: amd64.\e[m"
        return 1
    fi
}

setup_cuda_apt_repo() {
    local keyring_deb="/tmp/cuda-keyring_1.1-1_all.deb"
    local keyring_url="https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb"

    if dpkg -s cuda-keyring >/dev/null 2>&1; then
        echo "cuda-keyring already installed, skipping."
    else
        echo -e "\e[36mAdding the NVIDIA CUDA apt repository...\e[m"
        apt_install update
        apt_install install ca-certificates curl gnupg
        run_cmd curl -fsSL -o "${keyring_deb}" "${keyring_url}"
        run_cmd sudo dpkg -i "${keyring_deb}"
    fi

    apt_install update
}

detect_nvidia_driver_version() {
    # Prints the installed driver version, or nothing when no driver is found
    local version=""

    if command -v nvidia-smi >/dev/null 2>&1; then
        version="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1 | tr -d '[:space:]' || true)"
    fi

    # A driver installed but not yet loaded (no reboot yet) is only visible below
    if [[ -z "${version}" && -r /proc/driver/nvidia/version ]]; then
        version="$(awk '{ for (i = 1; i <= NF; i++) if ($i ~ /^[0-9]+\.[0-9]+/) { print $i; exit } }' /proc/driver/nvidia/version || true)"
    fi

    if [[ -z "${version}" ]] && command -v modinfo >/dev/null 2>&1; then
        version="$(modinfo nvidia -F version 2>/dev/null | head -1 | tr -d '[:space:]' || true)"
    fi

    printf '%s' "${version}"
}

install_nvidia_driver() {
    local installed_version
    local installed_branch

    if [[ "${option_no_cuda_drivers}" == true ]]; then
        echo -e "\e[33mSkipping NVIDIA driver (--no-cuda-drivers).\e[m"
        return 0
    fi

    installed_version="$(detect_nvidia_driver_version)"
    installed_branch="${installed_version%%.*}"

    if [[ "${installed_branch}" =~ ^[0-9]+$ ]]; then
        if ((installed_branch >= nvidia_driver_branch)); then
            echo -e "\e[32mNVIDIA driver ${installed_version} already installed (branch >= ${nvidia_driver_branch}), skipping.\e[m"
            return 0
        fi
        echo -e "\e[36mNVIDIA driver ${installed_version} is older than branch ${nvidia_driver_branch}; upgrading...\e[m"
    else
        echo -e "\e[36mNo NVIDIA driver detected; installing branch ${nvidia_driver_branch}...\e[m"
    fi

    apt_install install "cuda-drivers-${nvidia_driver_branch}"

    echo -e "\e[32mDone.\e[0m Driver ${nvidia_driver_branch} installed; reboot before running 'nvidia-smi'."
}

install_cuda_toolkit() {
    local -a cuda_packages

    echo -e "\e[36mInstalling CUDA Toolkit ${cuda_version} with cuBLAS / NPP / cuFFT...\e[m"

    if [[ "${option_runtime}" == true ]]; then
        cuda_packages=(
            "cuda-runtime-${cuda_apt_suffix}"
            "cuda-cudart-${cuda_apt_suffix}"
            "libcublas-${cuda_apt_suffix}"
            "libcufft-${cuda_apt_suffix}"
            "libnpp-${cuda_apt_suffix}"
        )
    else
        cuda_packages=(
            "cuda-toolkit-${cuda_apt_suffix}"
            "cuda-cudart-${cuda_apt_suffix}"
            "cuda-cudart-dev-${cuda_apt_suffix}"
            "libcublas-${cuda_apt_suffix}"
            "libcublas-dev-${cuda_apt_suffix}"
            "libcufft-${cuda_apt_suffix}"
            "libcufft-dev-${cuda_apt_suffix}"
            "libnpp-${cuda_apt_suffix}"
            "libnpp-dev-${cuda_apt_suffix}"
        )
    fi

    apt_install install "${cuda_packages[@]}"

    # Superseded by the OpenCV env below once that build succeeds
    nvidia_env_content="export CUDA_HOME=\"/usr/local/cuda\"
export PATH=\"\${CUDA_HOME}/bin:\${PATH}\"
export LD_LIBRARY_PATH=\"\${CUDA_HOME}/lib64\${LD_LIBRARY_PATH:+:\${LD_LIBRARY_PATH}}\""

    echo -e "\e[32mDone.\e[0m CUDA ${cuda_version} installed. Verify: /usr/local/cuda/bin/nvcc --version"
}

install_cudnn() {
    local -a cudnn_packages

    echo -e "\e[36mInstalling cuDNN 9 for CUDA ${cudnn_cuda_major}...\e[m"

    cudnn_packages=("libcudnn9-cuda-${cudnn_cuda_major}")
    if [[ "${option_runtime}" != true ]]; then
        cudnn_packages+=(
            "libcudnn9-dev-cuda-${cudnn_cuda_major}"
            "libcudnn9-headers-cuda-${cudnn_cuda_major}"
        )
    fi

    if ! apt-cache show "libcudnn9-cuda-${cudnn_cuda_major}" >/dev/null 2>&1; then
        echo -e "\e[33mSkipping cuDNN: 'libcudnn9-cuda-${cudnn_cuda_major}' not found in any apt source.\e[m"
        echo "Install the cuDNN local repo from https://developer.nvidia.com/cudnn and re-run."
        return 0
    fi

    apt_install install "${cudnn_packages[@]}"
    echo -e "\e[32mDone.\e[0m cuDNN installed."
}

install_tensorrt() {
    local -a tensorrt_packages
    local trt_major="${tensorrt_version%%.*}"
    local pinned_version

    echo -e "\e[36mInstalling TensorRT ${tensorrt_version}...\e[m"

    if [[ "${option_runtime}" == true ]]; then
        tensorrt_packages=(
            "libnvinfer${trt_major}"
            "libnvinfer-plugin${trt_major}"
            "libnvonnxparsers${trt_major}"
        )
    else
        tensorrt_packages=(
            "libnvinfer-dev"
            "libnvinfer-plugin-dev"
            "libnvonnxparsers-dev"
            "libnvinfer-headers-dev"
            "libnvinfer-bin"
        )
    fi

    if ! apt-cache show "${tensorrt_packages[0]}" >/dev/null 2>&1; then
        echo -e "\e[33mSkipping TensorRT: '${tensorrt_packages[0]}' not found in any apt source.\e[m"
        echo "TensorRT needs the NVIDIA local apt repo (login required):"
        echo "  1. Download 'nv-tensorrt-local-repo-ubuntu2404-${tensorrt_version}-cuda-*.deb' from"
        echo "     https://developer.nvidia.com/tensorrt"
        echo "  2. sudo dpkg -i nv-tensorrt-local-repo-*.deb && sudo apt update"
        echo "  3. Re-run: ./setup-dev-env.sh --nvidia"
        return 0
    fi

    # Pin every package to the same TensorRT release when that version is available.
    pinned_version="$(apt_candidate_version "${tensorrt_packages[0]}" "${tensorrt_version}")"
    if [[ -n "${pinned_version}" ]]; then
        echo "Pinning TensorRT packages to ${pinned_version}."
        local -a pinned_packages=()
        local pkg
        for pkg in "${tensorrt_packages[@]}"; do
            pinned_packages+=("${pkg}=${pinned_version}")
        done
        apt_install install "${pinned_packages[@]}"
    else
        echo -e "\e[33mTensorRT ${tensorrt_version} not available in apt; installing the default candidate.\e[m"
        apt_install install "${tensorrt_packages[@]}"
    fi

    echo -e "\e[32mDone.\e[0m TensorRT installed. Verify: dpkg -l | grep -E 'libnvinfer|libnvonnx'"
}

find_cuda_lib() {
    # find_cuda_lib <soname-stem>, searching cuda_lib_dirs and skipping stub libraries
    local name="$1"
    local dir
    local candidate

    for dir in "${cuda_lib_dirs[@]}"; do
        for candidate in "${dir}/lib${name}.so" "${dir}/lib${name}.so."*; do
            if [[ -e "${candidate}" && "${candidate}" != *"/stubs/"* ]]; then
                readlink -f "${candidate}"
                return 0
            fi
        done
    done

    return 1
}

ensure_unversioned_cuda_lib() {
    # CMake's FindCUDA needs lib<name>.so next to the versioned library
    local name="$1"
    local real_path
    local lib_dir

    real_path="$(find_cuda_lib "${name}" || true)"
    if [[ -z "${real_path}" ]]; then
        return 0
    fi

    lib_dir="$(dirname "${real_path}")"
    if [[ -e "${lib_dir}/lib${name}.so" ]]; then
        return 0
    fi

    if [[ -w "${lib_dir}" ]]; then
        ln -sfn "$(basename "${real_path}")" "${lib_dir}/lib${name}.so"
    else
        sudo ln -sfn "$(basename "${real_path}")" "${lib_dir}/lib${name}.so"
    fi
}

install_opencv_cuda() {
    local previous_dir="${PWD}"
    local install_prefix="${OPENCV_CUDA_PREFIX:-${HOME}/.local/opencv-cuda}"
    # Keep the CUDA object tree out of the workspace; only the source lives in dependency/.
    local build_root="${OPENCV_CUDA_BUILD_ROOT:-${HOME}/.cache/opencv_cuda_build}"
    local build_dir="${build_root}/build-${opencv_version}"
    local cuda_libs_root
    local cuda_lib_path
    local cuda_arch_bin
    local env_file
    local lib
    local -a generator
    local -a missing_libs

    # Only core + the contrib CUDA modules used by tensorrt_common
    local build_list="core,imgproc,imgcodecs,cudev,cudaarithm,cudawarping,cudaimgproc,cudafilters"

    echo -e "\e[36mBuilding OpenCV ${opencv_version} with CUDA...\e[m"

    apt_install install \
        build-essential \
        cmake \
        ninja-build \
        pkg-config \
        wget \
        unzip \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        libtbb-dev

    if [[ -z "${CUDA_HOME:-}" ]]; then
        if [[ -x /usr/local/cuda/bin/nvcc ]]; then
            export CUDA_HOME=/usr/local/cuda
        elif [[ -x "${HOME}/.local/cuda-${cuda_version}-root/usr/local/cuda-${cuda_version}/bin/nvcc" ]]; then
            export CUDA_HOME="${HOME}/.local/cuda-${cuda_version}-root/usr/local/cuda-${cuda_version}"
        fi
    fi

    if [[ -z "${CUDA_HOME:-}" || ! -x "${CUDA_HOME}/bin/nvcc" ]]; then
        echo -e "\e[33mSkipping OpenCV: nvcc not found (install the CUDA Toolkit first, or set CUDA_HOME).\e[m"
        return 0
    fi

    export PATH="${CUDA_HOME}/bin:${PATH}"

    # cuBLAS / NPP / cuFFT may live in a separate tree from the toolkit
    cuda_libs_root="${CUDA_LIBS_ROOT:-${CUDA_HOME}}"
    cuda_lib_dirs=("${CUDA_HOME}/lib64")
    if [[ -d "${cuda_libs_root}/targets/x86_64-linux/lib" ]]; then
        cuda_lib_dirs+=("${cuda_libs_root}/targets/x86_64-linux/lib")
    fi
    cuda_lib_path="$(IFS=:; echo "${cuda_lib_dirs[*]}")"
    export LD_LIBRARY_PATH="${cuda_lib_path}:${LD_LIBRARY_PATH:-}"
    export LIBRARY_PATH="${cuda_lib_path}:${LIBRARY_PATH:-}"

    for lib in cublas cublasLt cufft nppc nppial nppicc nppidei nppif nppig nppim nppist nppisu nppitc npps; do
        ensure_unversioned_cuda_lib "${lib}" || true
    done

    local cublas_lib cufft_lib
    local nppc_lib nppial_lib nppicc_lib nppidei_lib nppif_lib nppig_lib
    local nppim_lib nppist_lib nppisu_lib nppitc_lib npps_lib

    cublas_lib="$(find_cuda_lib cublas || true)"
    cufft_lib="$(find_cuda_lib cufft || true)"
    nppc_lib="$(find_cuda_lib nppc || true)"
    nppial_lib="$(find_cuda_lib nppial || true)"
    nppicc_lib="$(find_cuda_lib nppicc || true)"
    nppidei_lib="$(find_cuda_lib nppidei || true)"
    nppif_lib="$(find_cuda_lib nppif || true)"
    nppig_lib="$(find_cuda_lib nppig || true)"
    nppim_lib="$(find_cuda_lib nppim || true)"
    nppist_lib="$(find_cuda_lib nppist || true)"
    nppisu_lib="$(find_cuda_lib nppisu || true)"
    nppitc_lib="$(find_cuda_lib nppitc || true)"
    npps_lib="$(find_cuda_lib npps || true)"

    missing_libs=()
    [[ -n "${cublas_lib}" ]] || missing_libs+=("cublas")
    [[ -n "${cufft_lib}" ]] || missing_libs+=("cufft")
    [[ -n "${nppc_lib}" ]] || missing_libs+=("nppc")
    [[ -n "${npps_lib}" ]] || missing_libs+=("npps")

    if ((${#missing_libs[@]})); then
        echo -e "\e[33mSkipping OpenCV: missing CUDA libraries: ${missing_libs[*]}\e[m"
        echo "Expected under ${cuda_lib_path}"
        echo "Install libcublas-${cuda_apt_suffix} / libcufft-${cuda_apt_suffix} / libnpp-${cuda_apt_suffix}, or set CUDA_LIBS_ROOT."
        return 0
    fi

    cuda_arch_bin="${OPENCV_CUDA_ARCH_BIN:-}"
    if [[ -z "${cuda_arch_bin}" ]]; then
        cuda_arch_bin="$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null | head -1 | tr -d ' ' || true)"
        cuda_arch_bin="${cuda_arch_bin:-8.6}"
    fi

    echo "  CUDA_HOME       = ${CUDA_HOME}"
    echo "  CUDA_LIBS_ROOT  = ${cuda_libs_root}"
    echo "  CUDA_ARCH_BIN   = ${cuda_arch_bin}"
    echo "  SOURCE_DIR      = ${opencv_src_dir}"
    echo "  CONTRIB_DIR     = ${opencv_contrib_src_dir}"
    echo "  INSTALL_PREFIX  = ${install_prefix}"
    echo "  BUILD_DIR       = ${build_dir}"

    mkdir -p "${install_prefix}"
    ensure_opencv_source

    rm -rf "${build_dir}"
    mkdir -p "${build_dir}"
    cd "${build_dir}"

    if command -v ninja >/dev/null 2>&1; then
        generator=(-G Ninja)
    else
        generator=()
    fi

    cmake "${generator[@]}" \
        -D CMAKE_BUILD_TYPE=Release \
        -D CMAKE_INSTALL_PREFIX="${install_prefix}" \
        -D CMAKE_CXX_STANDARD=17 \
        -D CMAKE_PREFIX_PATH="${cuda_libs_root}/targets/x86_64-linux" \
        -D CMAKE_LIBRARY_PATH="${cuda_lib_path}" \
        -D OPENCV_EXTRA_MODULES_PATH="${opencv_contrib_src_dir}/modules" \
        -D BUILD_LIST="${build_list}" \
        -D WITH_CUDA=ON \
        -D WITH_CUDNN=ON \
        -D OPENCV_DNN_CUDA=OFF \
        -D ENABLE_FAST_MATH=ON \
        -D CUDA_FAST_MATH=ON \
        -D WITH_CUBLAS=ON \
        -D WITH_CUFFT=ON \
        -D CUDA_ARCH_BIN="${cuda_arch_bin}" \
        -D CUDA_ARCH_PTX="" \
        -D CUDA_TOOLKIT_ROOT_DIR="${CUDA_HOME}" \
        -D CUDA_NVCC_FLAGS="-std=c++17" \
        -D CMAKE_CUDA_STANDARD=17 \
        -D CMAKE_CXX_FLAGS="-I${cuda_libs_root}/targets/x86_64-linux/include" \
        -D CUDA_cublas_LIBRARY="${cublas_lib}" \
        -D CUDA_cufft_LIBRARY="${cufft_lib}" \
        -D CUDA_nppc_LIBRARY="${nppc_lib}" \
        -D CUDA_nppial_LIBRARY="${nppial_lib}" \
        -D CUDA_nppicc_LIBRARY="${nppicc_lib}" \
        -D CUDA_nppidei_LIBRARY="${nppidei_lib}" \
        -D CUDA_nppif_LIBRARY="${nppif_lib}" \
        -D CUDA_nppig_LIBRARY="${nppig_lib}" \
        -D CUDA_nppim_LIBRARY="${nppim_lib}" \
        -D CUDA_nppist_LIBRARY="${nppist_lib}" \
        -D CUDA_nppisu_LIBRARY="${nppisu_lib}" \
        -D CUDA_nppitc_LIBRARY="${nppitc_lib}" \
        -D CUDA_npps_LIBRARY="${npps_lib}" \
        -D CUDNN_INCLUDE_DIR=/usr/include/x86_64-linux-gnu \
        -D CUDNN_LIBRARY=/usr/lib/x86_64-linux-gnu/libcudnn.so \
        -D WITH_GTK=OFF \
        -D WITH_QT=OFF \
        -D WITH_OPENGL=OFF \
        -D BUILD_EXAMPLES=OFF \
        -D BUILD_TESTS=OFF \
        -D BUILD_PERF_TESTS=OFF \
        -D BUILD_opencv_apps=OFF \
        -D BUILD_opencv_python2=OFF \
        -D BUILD_opencv_python3=OFF \
        -D BUILD_JAVA=OFF \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        -D OPENCV_ENABLE_NONFREE=OFF \
        "${opencv_src_dir}"

    echo "Building (this can take a while)..."
    if command -v ninja >/dev/null 2>&1; then
        ninja -j"$(nproc)"
        ninja install
    else
        make -j"$(nproc)"
        make install
    fi

    if [[ ! -f "${install_prefix}/include/opencv4/opencv2/cudaimgproc.hpp" ]]; then
        echo -e "\e[33mWarning: cudaimgproc.hpp is missing; the OpenCV CUDA modules were not built.\e[m"
    fi

    # Same content lands in ~/.bashrc and in the sourceable env file, with $HOME left unexpanded
    local prefix_expr cuda_home_expr cuda_lib_path_expr
    prefix_expr="$(portable_home_path "${install_prefix}")"
    cuda_home_expr="$(portable_home_path "${CUDA_HOME}")"
    cuda_lib_path_expr="$(portable_home_path_list "${cuda_lib_path}")"

    nvidia_env_content="export CUDA_HOME=\"${cuda_home_expr}\"
export PATH=\"\${CUDA_HOME}/bin:\${PATH}\"
export LD_LIBRARY_PATH=\"${prefix_expr}/lib:${cuda_lib_path_expr}\${LD_LIBRARY_PATH:+:\${LD_LIBRARY_PATH}}\"
export OpenCV_DIR=\"${prefix_expr}/lib/cmake/opencv4\"
export PKG_CONFIG_PATH=\"${prefix_expr}/lib/pkgconfig\${PKG_CONFIG_PATH:+:\${PKG_CONFIG_PATH}}\""

    env_file="${install_prefix}/setup_env.sh"
    {
        echo "# Generated by setup-dev-env.sh --nvidia"
        echo "${nvidia_env_content}"
    } > "${env_file}"

    cd "${previous_dir}"

    echo -e "\e[32mDone.\e[0m OpenCV ${opencv_version} installed into ${install_prefix}."
    echo "Source tree: ${opencv_src_dir}"
    echo "Sourceable copy of the same env: ${env_file}"
}

install_nvidia_stack() {
    if [[ "${option_no_nvidia}" == true ]]; then
        echo -e "\e[33mSkipping the NVIDIA stack (--no-nvidia).\e[m"
        return 0
    fi

    if ! check_nvidia_prerequisites; then
        return 0
    fi

    setup_cuda_apt_repo
    install_nvidia_driver
    install_cuda_toolkit
    install_cudnn
    install_tensorrt
    install_opencv_cuda

    if [[ -n "${nvidia_env_content}" ]]; then
        write_bashrc_block "asr_sdm nvidia env" "${nvidia_env_content}"
    fi

    echo -e "\e[32mNVIDIA stack setup complete.\e[0m"
    echo "Reload: source ~/.bashrc  |  Reboot first if the driver was installed."
}

parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -h | --help)
                print_help
                exit 0
                ;;
            --list)
                option_list=true
                shift
                ;;
            --common)
                option_common=true
                shift
                ;;
            --ros)
                option_ros=true
                shift
                ;;
            --dependency)
                option_dependency=true
                shift
                ;;
            --nvidia)
                option_nvidia=true
                shift
                ;;
            --no-nvidia)
                option_no_nvidia=true
                shift
                ;;
            --no-cuda-drivers)
                option_no_cuda_drivers=true
                shift
                ;;
            --runtime)
                option_runtime=true
                shift
                ;;
            -y)
                option_yes=true
                shift
                ;;
            -v | --download-artifacts)
                echo "Warning: option '$1' is not implemented yet, ignoring."
                shift
                ;;
            --ros-distro)
                if [[ $# -lt 2 ]]; then
                    echo "Error: option '$1' requires an argument."
                    print_help
                    exit 1
                fi
                if [[ "$2" != "${ros_distro}" ]]; then
                    echo "Error: unsupported ROS distribution '$2'. Only '${ros_distro}' is supported."
                    exit 1
                fi
                shift 2
                ;;
            --data-dir | --module)
                if [[ $# -lt 2 ]]; then
                    echo "Error: option '$1' requires an argument."
                    print_help
                    exit 1
                fi
                echo "Warning: option '$1' is not implemented yet, ignoring."
                shift 2
                ;;
            *)
                echo "Error: unknown option '$1'"
                print_help
                exit 1
                ;;
        esac
    done
}

main() {
    parse_args "$@"

    if [[ "${option_list}" == true ]]; then
        print_list
        exit 0
    fi

    if [[ "${option_common}" != true && "${option_ros}" != true \
        && "${option_dependency}" != true && "${option_nvidia}" != true ]]; then
        print_help
        echo "No actionable option provided. Use --list, --common, --ros, --nvidia and/or --dependency."
        exit 1
    fi

    confirm_or_exit

    if [[ "${option_common}" == true ]]; then
        install_common
    fi

    if [[ "${option_ros}" == true ]]; then
        install_ros
    fi

    if [[ "${option_nvidia}" == true ]]; then
        install_nvidia_stack
    fi

    if [[ "${option_dependency}" == true ]]; then
        install_dependency
    fi
}

main "$@"
