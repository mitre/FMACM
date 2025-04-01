FROM artifacts.mitre.org:8200/docker/idealab/centos7/idealab-basic:1.0.2 AS build
ENV container docker

# Set the TZ
ENV TZ=America/New_York
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
# ENV CPM_SOURCE_CACHE=$HOME/.cache/CPM

RUN yum install -y zsh && \
    zsh -c 'git clone --recursive https://github.com/sorin-ionescu/prezto.git "${ZDOTDIR:-$HOME}/.zprezto"' && \
    zsh -c 'setopt EXTENDED_GLOB; for rcfile in "${ZDOTDIR:-$HOME}"/.zprezto/runcoms/^README.md(.N); do ln -s "$rcfile" "${ZDOTDIR:-$HOME}/.${rcfile:t}"; done' && \
    chsh -s /bin/zsh

# ----------------------------------------------------------------------------
# AAESim-specific artifacts are pulled into the container below

# BUILD DEPENDENCIES---

# RUN DEPENDENCIES---
# BADA data (needed by AAESim/FMACM in order to run a scenario)
RUN mkdir /data \
    && mkdir /data/aaesim \
    && mkdir /data/aaesim/regressionScens \
    && git clone -b master --depth 1 https://mustache.mitre.org/scm/aaes/bada-data.git /data/aaesim/regressionScens/bada

# TEST DEPENDENCIES---
# NOTE: you only need this repo if you intend to run aaesim unit/regression tests inside the container
# BEWARE: this clone operation takes a long time for some reason
# RUN git clone -b main --depth 1 https://mustache.mitre.org/scm/aaes/regression_input_data.git /data/aaesim/regression_input_data 


# ----------------------------------------------------------------------------
# Final configuration steps

# Make sure sshd is running (clion uses this for communication)
CMD ["/usr/sbin/sshd", "-D", "-e", "-f", "/etc/ssh/sshd_config"]  
