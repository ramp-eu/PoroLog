# PoRoLog ROSE-AP

[![License: Apache 2.0](https://img.shields.io/github/license/Factobotics/FlexHex-Rose-AP)](https://opensource.org/licenses/Apache-2.0)
<br/>

ROSE-AP of the PoRoLog project, offers
- Easy deployment of the NOOS Open Cloud infrastructure via `docker-compose`, allowing for robot logistics applications
- Bootstrapping ROS2 software that can be installed in a ROS2-enabled Robot to automatically connect it to the NOOS Open infrastructure
- [Codin](https://codin.issel.ee.auth.gr) dashboards, via which a Warehouse that uses the NOOS Open infrastructure can inspect its real-time status.

This project is part of [DIH^2](http://www.dih-squared.eu/).

## Contents

-   [Background](#background)
-   [Installation guide (NOOS-Open)](#noos-open-installation)
    - [Deployment of NOOS-Open](#deployment-of-noos-open-cloud-infrastructure)
    - [Prerequisites](#prerequisites)
    - [Setup](#setup)
    - [Execution](#execution)
    - [The Codin dashboard](#codin-dashboard)
    - [Future work](#future-work)
-   [Robot(s) setup](#robots-setup)
    - [Bootstrapping software](#bootstrapping-software)
    - [Custom robot integration](#custom-robot-integration)
-   [Step by step tutorial](#step-by-step-tutorial)
-   [Additional utilities](#fiware--mqtt-broker-accompanying-files)

# Background

Modern warehouses require for modern logistics solutions. PoRoLog aspires to provide a robotics-based solution that can contribute to automate and/or improve day-to-day workload, so that the warehouse improves its efficiency, optimizes storage capacity, and decrease physical work. Specifically, PoRoLog proposes a new reconfigurable and scalable robotic system for pallet and box transfers in existing logistics warehouses and a new open source software for supervision of groups of robots named NOOS-Open. A real size technology transfer experiment in a real warehouse with Industry and Logistics 4.0 in mind will be performed, resulting in hardware and software available for other SMEs on RAMP for easy replication at European level.

---

# NOOS-Open Installation

## Deployment of Noos-Open cloud infrastructure

Noos-Open is supported by a cloud infrastructure, which relies on a [Fiware Orion broker](https://fiware-orion.readthedocs.io/en/master/) and a [Mosquitto MQTT broker](https://mosquitto.org/) instance. In the following image, you may see the overall architecture.

![Untitled](https://user-images.githubusercontent.com/5663091/195070228-ba4e176c-88c8-40e0-8b55-09cc5d6fcb2a.png)

The deployment of the cloud components of Noos-Open can be achieved using these files: https://github.com/robotics-4-all/porolog-deployment

**Important: It is presumed that all commands are executed from the root directory of this repository!**

## Prerequisites

In order to properly deploy this infrastructure the following packages are required

| Software       | Version                                |
| -------------- | -------------------------------------- |
| docker         | 19.03.5-ce, build 633a0ea838 or higher |
| docker-compose | 2.4.1 or higher                        |

- In order to install docker engine follow the [tutorial][docker_tutorial].
- In order to install docker-compose follow the [tutorial][docker_compose_tutorial].

## Setup

In order to properly configure the **setup the environmental variables** of .env file must be set approprietly. Also the mosquitto **pwfile file must be overridden** with the appropriate credentials and be placed in the ./mosquitto directory.

To create a mosquitto pwfile simply run:

```sh
mosquitto_passwd -U ./mosquitto/pwfile <user>
```

Note: You will be prompted to enter the user's password twice.

The **MOSQUITTO_USERNAME & MOSQUITTO_USERNAME** must in the .env file be updated accordingly to match the credentials given in the pwfile so as the json-iot-agent is able to connect to the mosquitto broker. Optionally the other environmental variables can also be changed.

Finally the **host** environmental variable must be set to must the **domain_name/ip** of the host machine.

## Execution

To create the infrastructure simply execute the following command:

```sh
./launch.sh
```

Note: The script must have execution privilages. Run the following command as superuser if the priviledges are insufficient.

```sh
sudo chmod +x launch.sh
```

## Codin dashboard

Codin is not open-source but it is (and will be) free for use. The dashboards can be exported in JSON formatted files and imported by another user, making the solution easily transferable. 

The Codin dashboard created for supporting Noos-Open can be found in [this link](https://github.com/ortelio/Noos-Open/blob/main/Porolog%20showcasing.json). You can import it in [Codin](https://codin.issel.ee.auth.gr/), declare your deployment's credentials and inspect your robots!

## Future work
- Security on orion context broker via keyrock
- Mosquitto certificates

[docker_tutorial]: https://docs.docker.com/engine/install/ubuntu/
[docker_compose_tutorial]: < https://docs.docker.com/compose/install/>

---

# Robot(s) setup

In order to integrate a robotic platform to the NOOS-Open infrastructure, the robot must be ROS2-enabled, at least supporting the ROS2 Navigation stack. Furthermore, its OS must be unix-based, in order to be able to execute Python v3, as well as the [Commlib-py](https://pypi.org/project/commlib-py/) library, which acts as the communication channel between the Robot and NOOS-Open.

## Bootstrapping software

For easier integration, NOOS-Open provides a bootstrapping ROS2 package that can be easily modified for any ROS2-supporting robot and allow for quick and easy integration with the NOOS-Open infrastructure. This package is located [here]().

The instructions to execute it follow:
> VASILIS HERE!

## Custom robot integration

In the case where anyone wants to implement their own controllers (or some of them), they can write their own Python ROS2 package and create the following Commlib publishers/subscribers with the following data models:

#### PUBLISHER `/<ORION_API_KEY>/<ROBOT_ID>/attrs/battery`
```yaml
{
    percentage: <Float>
}
```

#### PUBLISHER `/<ORION_API_KEY>/<ROBOT_ID>/attrs/image`
```yaml
{
    val: <String> # The base64 encoded image
}
```

#### PUBLISHER `/<ORION_API_KEY>/<ROBOT_ID>/attrs/logs`
```yaml
{
    val: <String>
}
```

#### PUBLISHER `/<ORION_API_KEY>/<ROBOT_ID>/attrs/state`
```yaml
{
    state: <String>
}
```

#### PUBLISHER `/<ORION_API_KEY>/<ROBOT_ID>/attrs/status`
```yaml
{
    status: <String>
}
```

#### SUBSCRIBER `/<ORION_API_KEY>/<ROBOT_ID>/attrs/velocities`
```yaml
{
    command: <String> # One of FORWARD, BACKWARD, LEFT, RIGHT, STOP, LIFT, RELEASE
}
```

#### PUBLISHER `/<ORION_API_KEY>/<ROBOT_ID>/attrs/status`
```yaml
{
    linear: <Float>,
    angular: <Float>
}
```

#### PUBLISHER `/<ORION_API_KEY>/<ROBOT_ID>/attrs/status`
```yaml
{
    linear: <Float>,
    angular: <Float>
}
```

#### PUBLISHER `/<ORION_API_KEY>/<ROBOT_ID>/attrs/pose`
```yaml
{
    x: <Float>,
    y: <Float>,
    theta: <Float>
}
```

#### PUBLISHER `/<ORION_API_KEY>/<ROBOT_ID>/attrs/path`
```yaml
{
    path: [
        {
            x: <Float>,
            y: <Float>
        },
        ...
    ]
}
```

#### SUBSCRIBER `/<ORION_API_KEY>/<ROBOT_ID>/attrs/target`
```yaml
{
    x: <Float>,
    y: <Float>
}
```

#### SUBSCRIBER `/<ORION_API_KEY>/<ROBOT_ID>/attrs/goal/entity`
```yaml
{
    type: <String>, # One of Parcel, Pallet, Slot
    id: <String>
}
```

---

# Step by step tutorial

The step by step tutorial will be uploaded to YouTube and the link will be posted here!

---

# Utilities - Fiware & MQTT broker accompanying files

- [Commlib-py](https://pypi.org/project/commlib-py/): Robot communication controller, i.e. the software to dispatch information to Fiware broker via MQTT, using Python. This component is commlib-py and is currently open-source
- [Fiware entities JSON](https://github.com/ortelio/Noos-Open/blob/main/fiware-entities.tar.xz): Here you can find a collection of JSON-formatted files that contain the Fiware NGSI types, suitable for logistics applications
- [Hoppscotch Fiware-related REST calls](https://github.com/ortelio/Noos-Open/blob/main/hoppscotch_calls.zip): REST collections for invoking and updating all Fiware entities. Hoppscotch is the open-source alternative of Postman, alleviating its restrictions regarding membership
- [Script](https://github.com/ortelio/Noos-Open/blob/main/fiware_bootstrapping.py): Python script using [REST-ee-Fi](https://github.com/robotics-4-all/fiware-ngsi-api), via which an initial insertion of mock data to the respective Fiware entities is performed, so as for the Hoppscotch calls to operate
- [Script](https://github.com/ortelio/Noos-Open/blob/main/fiware_parcel_2d_transformation.py): Python script that takes as input a Parcel’s ID and retrieves its x,y coordinates in the absolute coordinate frame (warehouse frame)


