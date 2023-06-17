# Robothon Grand-Challenge documentation template
**Table of content:**
- [Robothon Grand-Challenge documentation template](#robothon-grand-challenge-documentation-template)
  - [The bare minimum of documentation of any robotic project should include:](#the-bare-minimum-of-documentation-of-any-robotic-project-should-include)
  - [The exeptional good documentation also includes:](#the-exeptional-good-documentation-also-includes)
  - [Solution overview](#solution-overview)
  - [Programs](#programs)
    - [Hardware dependencies](#hardware-dependencies)
    - [Software dependencies](#software-dependencies)
  - [How to run](#how-to-run)
  - [Solution in-dept description](#solution-in-dept-description)
  - [Authors](#authors)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>



## The bare minimum of documentation of any robotic project should include:

TODO

- [ ] A solution overview
- [ ] A software and hardware dependencie list of any kind
- [ ] A how to build, run, and get started section
- [ ] An Author list with contact infomation
- [ ] Citations to inspired works and dependencies

And will make you able to run the solution on a identiacly workcell without prior knowlegde of your solution. See it as it will make yourself able to use your solution again in 2 years.

## The exeptional good documentation also includes:
- [ ] An indepth solution description
- [ ] A fully functional virtualised developer environment (Like a docker img etc.)
- [ ] Is tested by somone external
- [ ] A how and where to contribute section

## Solution overview

TODO


The good solution overview has:
- [ ] A picture of the workcell and solution
- [ ] A diagram of software and hardware components and their connections
- [ ] A small description, what is developed, most interesting with this solution and what is it inspired by.

## Programs

- **[FO][P]** Touch button
- **[FO][P]** Grip probe plug
- **[FC][P]** Insert probe plug
- **[P]** Grab slider
- **[FO][P]** Move slider
- **[FC][V]** Open Door
- **[FO][P]** Pick probe (box)
- **[FC][P][V]** Place probe (box)
- **[FO][P]** Pick probe (holder)
- **[FC][P][V]** Place probe (holder)
- **[FC][P]** Contact probe
- **[P]** Cableing

### Hardware dependencies

| Hardware type     | Model              | OS/Driver version | Note/Picture                                                                                          |
|-------------------|--------------------|-------------------|-----------------------------------------------------------------------------------------------|
| Robot             | Universal robot - UR5e | SW 5.13.1         | [Link for official site](https://www.universal-robots.com/products/ur5-robot/)                |
| 3D printed finger | Costume made       | N/A                | TODO |
| Computer          | IPC Nuvo-5002E    | ubuntu 20.04  |                                                                                               |
| Camera            |         None           |                   |                                                                                               |
| Gripper           |    Weiss Robotic WSG-50-110                |        TODO           |        [Link](https://weiss-robotics.com/de/wsg-series/product/wsg-serie/selectVariant/wsg-50-110/)                                                    |
### Software dependencies
The good software depenency list has:
- [ ] Listed all packages, software etc. needed to run the solution!
- [ ] Version or git commit used in project
- [ ] Link to where/how it can be aquired

| Name          | Version / git commit / placement in repository                                  | What                                                 | Note (Third-party, commercial, homemade etc.) |
|---------------|---------------------------------------------------------------------------------|------------------------------------------------------|-----------------------------------------------|
| pitasc       | [See here](https://www.pitasc.fraunhofer.de/)                                                                          | Robot programming framework                 | Internal                                  |

## How to run
The good how to build, run and get started section has:
- [ ] Start pitasc
- [ ] Run programs with `pi_teacher` in rqt

## Solution in-dept description
The good how to build, run and get started section has:
- [ ] Descripes the solution and all it's interesting parts a bit more in depth, how it can be modified and what their porpuse is.



## Authors
The good authors section has:
- [ ] Listed all authors and a way to get in contact with them.

 Daniel Bargmann, FhG IPA, [Github](https://github.com/ipa-danb), [danb@ipa.fraunhofer.de](mailto:danb@ipa.fraunhofer.de)