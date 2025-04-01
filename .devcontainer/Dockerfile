FROM artifacts.mitre.org:8200/idealab/rocky/idealab-basic:2.1.0
ENV container docker

# Set the TZ
ENV TZ=America/New_York
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN dnf install -y ccache

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
COPY --from=artifacts.mitre.org:8200/docker/sbowman/aaesim_regression_data:latest /data/aaesim/regression_input_data /data/aaesim/regression_input_data

# ----------------------------------------------------------------------------
# Final configuration steps
RUN dnf -y install zsh dotnet-sdk-7.0 aspnetcore-runtime-7.0

# Make sure sshd is running (clion uses this for communication)
CMD ["/usr/sbin/sshd", "-D", "-e", "-f", "/etc/ssh/sshd_config"]  

