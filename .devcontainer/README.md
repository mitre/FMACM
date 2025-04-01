# AAESim Development Container

This folder provides docker containers that allow local development of aaesim (even for IDEs like VSCode & CLion). Deploy containers are also here.

## Developer Images

### To Build

Let's do this from the command line. First, build the image to use locally:

`>> docker build -t aaesim-devel .`

### To Run

Run the docker container locally. This will create a running container called `aaesim-dev-sandbox`. In docker hub, you'll see it pop up as a running container.

`>> docker run -d --cap-add sys_ptrace -p127.0.0.1:2222:22 --name aaesim-dev-sandbox aaesim-devel`

If you want to stop the container:

`>> docker stop aaesim-dev-sandbox`

### Remote Debugging

#### VSCode

`vscode` can run a local container and host your code in that container. There are different ways to do this and you should [get smart](https://code.visualstudio.com/docs/containers/overview) before diving in here.

TL;DR: On your host system, open the root repository folder (e.g. `aaesim`) in `vscode`. The IDE will read the file `.devcontainer/devcontainer.json` and ask you if you'd like to open your code in the container defined in that file. If you select yes, `vscode` will do some magic to start the container and mount your code in the container. From there, you can develop, debug, & commit with happiness using launch and task settings.

Some `vscode` extensions are necessary for this: ms-azuretools.vscode-docker, ms-vscode-remote.remote-containers, ms-vscode-remote.remote-ssh.

#### CLion

In CLion, setup a remote toolchain using `localhost` and port `2222`. The user will be `user` and the password is `password`. This means that all operations performed by CLion in your container will be performed as `user` not `root`.

For more information about running a containerized dev environment for clion, see also [their official blog post](https://blog.jetbrains.com/clion/2020/01/using-docker-with-clion/).

### Data Access

It is possible to run the container and also connect a volume mount from your host machine. This can be used to allow transparent transfer of data between your host and your container. So, if you need to provide quick access to data that exists on your host machine, just make it available in that shared volume.

**Note**: symlinks created in the volume mount on the host machine that point to external locations on the host machine may not be visible to your container.

#### For AAESim

`aaesim` needs plenty of data at run-time in order to do anything interesting: BADA data files, RUC/RAP wind files, etc. In the IDEA Lab, this data exists conveniently on the NFS. This container resolves enough of the external data needs to facilitate running unit and regression tests. When you build the image, docker pulls in the git repos that contains the BADA data files and also the wind files necessary for testing. You can examine this yourself: once inside your container, you can find these data files in `/data/aaesim/` (which mimics the location on the NFS).

All other data needs are up to you to provide. Wind data, beyond the few files needed for testing, is huge and does not exist in a git repo. Nor would you want it all on your local machine anyway! One possibility is to cherry-pick the files you need to run with and provide them to your docker container via a local volume mount. That is up to you.

## Deployable Containers

Deployed images are [available on internal Artifactory](https://artifacts.mitre.org/ui/repos/tree/General/docker-local/docker/aaesim). Generally only releases are deployed and those releases are automated on [Bamboo](https://pandafood.mitre.org/browse/AAES-AAESTAGS). These polished containers allow downstream users to run aaesim on their local computer.

Build the deployable container. The docker file is parameterized with `ARG_AAESIM_GIT_TAG` to allow building _any_ branch or tag using a command line variable. Use this kind of command:

For local runs on `arm64`:

```bash
docker buildx build --platform linux/arm64 --load --build-arg ARG_AAESIM_GIT_TAG=v5.0.1 -t aaesim:5.0.1 -f Dockerfile.deploy_v5 .
```

For deployable images:

```bash
docker buildx build --platform linux/arm64,linux/amd64 --load --build-arg ARG_AAESIM_GIT_TAG=v5.0.1 -t aaesim:5.0.1 -f Dockerfile.deploy_v5 .
docker tag aaesim:5.0.1 artifacts.mitre.org:8200/aaesim/aaesim:5.0.1
docker push artifacts.mitre.org:8200/aaesim/aaesim:5.0.1
```

With no command line arguments provided, the container will run aaesim and self-report the build details.

```bash
docker run --rm -it artifacts.mitre.org:8200/aaesim/aaesim:5.0.1
```

To execute aaesim using a configuration that is stored on the local host computer, use volume mounts. This example mounts the current directory `pwd` to run a configuration file there:

```bash
docker run --rm -it -v `pwd`:`pwd` -w `pwd` artifacts.mitre.org:8200/aaesim/aaesim:5.0.1 ./<config-file>
```

The above will execute aaesim using the configuration file specified. All data produced by aaesim will be written back to the host computer in the current directory.

> **Note**: to run aaesim successfully requires input data. Bada data is already in the container at `/data/aaesim/regressionScens/bada`; use that path in your aaesim scenarios. All other input data (e.g. wind grb files) must be provided by volume mounts in the docker command line arguments!

### Minimum Run Container

The deployable containers are kept as small as possible, especially with respect to the supporting Linux packages. To facilitate this, the `Dockerfile.minimal` exists. It builds the smallest possible _base_ container for hosting an AAESim binary to run inside. `Dockerfile.minimal` does not ever contain an aaesim binary in it.

This container must be built using a multi-platform approach.

```bash
docker buildx build --platform linux/arm64,linux/amd64 --push -t artifacts.mitre.org:8200/aaesim/aaesim-minimal:2.2.0 -f Dockerfile.minimal .
```

Please be sure to update the version whenever a new `minimal` image is deployed to Artifactory.

- Major: indicates an OS change
- Minor: indicates progress/maintenance within an OS definition
- Patch: rarely used. Probably 0.

## Earlier Version Builds

While not generally needed, it is possible to still build earlier versions releases in an image. Select the correct docker file for your needs.

- v3 releases: `Dockerfile.deploy_v3`
- v4 releases: `Dockerfile.deploy_v4`
- v5 releases: `Dockerfile.deploy_v5`

----

~Happy Developing~
