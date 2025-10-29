FROM rockylinux:9 AS build
ENV container=docker

# ----------------------------------------------------------------------------
# Internal MITRE only...comment out otherwise
# ----------------------------------------------------------------------------
RUN curl -ksSL \
    https://gitlab.mitre.org/mitre-scripts/mitre-pki/raw/master/os_scripts/install_certs.sh \
    | MODE=centos sh

# Set the TZ
ENV TZ=America/New_York
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN dnf -y install epel-release dnf-plugins-core
RUN dnf -y install initscripts \
        ccache \
        git \
        gcc \
        gcc-c++ \
        cmake \
        make \
        wget \
        tar \
        netcdf-devel \
        netcdf-cxx \
        libaec \
        eccodes
RUN dnf --enablerepo=devel install -y eccodes-devel

# ----------------------------------------------------------------------------
# AAESim-specific artifacts are pulled into the container below

# BUILD DEPENDENCIES---

# RUN DEPENDENCIES---

# TEST DEPENDENCIES---

# ----------------------------------------------------------------------------
# Final configuration steps
# RUN dnf -y install zsh dotnet-sdk-7.0 aspnetcore-runtime-7.0

# Make sure sshd is running (clion uses this for communication)
# CMD ["/usr/sbin/sshd", "-D", "-e", "-f", "/etc/ssh/sshd_config"]  

