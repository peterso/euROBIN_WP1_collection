# ddeploy: a Docker-based deployment tool

**Table of content:**

- [Solution overview](#solution-overview)
  - [Software dependencies](#software-dependencies)
- [How to run](#how-to-run)
- [Authors](#authors)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>

## Solution overview

This is a tool for automating the deployment of ROS-based applications as Docker containers.
It can help build an _image_ with the target project and all its dependencies installed.

The build process consists of the following steps:

- Set up project sources in docker build context _in the host_
- Install dependencies inside image using `rosdep`
- Build and install project packages into an overlay of the chosen ROS distro

### Software dependencies

- Python 3.8

Python Packages:

- catkin-pkg: 0.4.23

- distro: 1.6.0

- docutils: 0.17.1

- pyparsing: 2.4.7

- python-dateutil: 2.8.2

- pyyaml: 5.3.1

- rosdistro: 0.8.3

- rospkg: 1.3.0

- six: 1.16.0

- vcstool: 0.2.15

All can be installed using the requirements.txt file:

```(bash)
pip install -r requirements.txt
```

_Note, however, that the above command will install `ddeploy` and its dependencies in a user-wide location, which may conflict with other package installations. Using a [Python virtual environment](https://docs.python.org/3/library/venv.html) is recommended._

## How to run

The minimum number of arguments to build an _image_ from packages present in the host is, e.g.:

```bash
ddeploy --project-name my_project --ros-distro melodic --project-sources '.'
```

or using the short alternatives:

```bash
ddeploy -n my_project -d melodic -s '.'
```

- `-n/--project-name` is used (together with the ROS distro) to generate the default _image_ and _addon_ names (more on this below).
- `-d/--ros-distro` is used to select which ROS distro the packages should be deployed against.
- `-s/--project-sources` is used to define the packages to be deployed and is described below.

Alternatively, a YAML file can be provided instead of supplying the options via command line arguments:

```bash
ddeploy --yaml my_project.ddeploy.yaml
```

Where `my_project.ddeploy.yaml` can contain, to be equivalent to the command line version above:

```yaml
project_name: my_project
ros_distro: melodic
project_sources: [.]
```

If both a YAML file and command line arguments are provided, the YAML file is read first, and its values are overwritten if matching command line arguments are provided.

## Authors

 Miguel Prada Sarasola,  [Github](https://github.com/miguelprada), [miguel.prada@tecnalia.com](mailto:miguel.prada@tecnalia.com)

 Jon Azpiazu Lozano,  [Github](https://github.com/jonazpiazu),[miguel.prada@tecnalia.com](jon.azpiazu@tecnalia.com)

 Iñigo Moreno i Caireta,  [inigo.moreno@tecnalia.com](mailto:inigo.moreno@tecnalia.com)
