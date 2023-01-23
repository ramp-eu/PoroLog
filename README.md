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
-   [Fiware NGSI types](#fiware-ngsi-types)
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

# Fiware NGSI types

In order for the NOOS open to correctly operate, logistics-specific NGSI data types have been defined, so as to handle the data storage and retrieval. Specifically we have the following types:

- **Warehouse**: Holds the necessary information of a whole warehouse.
- **WarehouseKPI**: Includes some Key Performance Indicators (KPIs) for the warehouse, for a single day. It is expected that a different WarehouseKPI instance is created each day.
- **Robot**: Holds the necessary information about a robot that exists in a Warehouse.
- **RobotKPI**: Holds the KPIs of a specific Robot.
- **Room**: A Warehouse consists of Rooms
- **Rack**: A Room contains Racks
- **Shelf**: A Rack contains Shelfs
- **Slot**: A Shelf contains Slots
- **Pallet**: A Pallet can be placed in a Slot
- **Parcel**: A Parcel can be placed in a Slot or exist in a Pallet



## Warehouse NGSI type

```yaml
{
    "id": "-",
    "type": "Warehouse",
    "name": {
        "type": "Text",
        "value": "Bleujour"
    },
    "georeference": {
        "type": "vector",
        "value": {
            "longitude": 43.5746458,
            "lattitude": 1.4514467
        },
        "metadata": {
            "address": {
                "type": "postalAddress",
                "value": {
                    "streetAddress": "37 Av. Jules Julien",
                    "addressRegion": "Toulouse, France",
                    "addressLocality": "-",
                    "postalCode": "31400"
                }
            }
        }
    },
    "blueprint": {
        "type": "string",
        "value": ""
    },
    "dimensions": {
        "type": "vector",
        "value": {
            "width": 100,
            "height": 50,
            "resolution": 0.1
        }
    },
    "annotations": {
        "type": "list",
        "value": []
    },
}
```

## WarehouseKPI NGSI type

```yaml
{
    "id": "-",
    "type": entity_type,
    "date": {
        "type": "Date",
        "value": {
            "val": "14/06/2022"
        }
    },
    "palleteStorageDensity": {
        "type": "number",
        "value": {
            "val": 1
        }
    },
    "distanceXmassMovedByRobots": {
        "type": "number",
        "value": {
            "val": 1
        }
    },
    "distanceXmassMovedByOperators": {
        "type": "number",
        "value": {
            "val": 1
        }
    },
    "palletsMoved": {
        "type": "number",
        "value": {
            "val": 1
        }
    },
    "parcelsMoved": {
        "type": "number",
        "value": {
            "val": 1
        }
    },
    "savedTimeshareForOperator": {
        "type": "number",
        "value": {
            "val": 1
        }
    },
    "refWarehouse": {
        "type": "Relationship",
        "value": "urn:ngsi-ld:Warehouse:1"
    }
}
```

## Robot NGSI type

## RobotKPI NGSI type

```yaml
{
    "id": "-",
    "type": entity_type,
    'date': {
        'type': 'Date',
        'value': {
            'val': ''
        }
    },
    'boxedMoved': {
        'type': 'number',
        'value': {
            'val': 0
        }
    },
    'palletesMoved': {
        'type': 'number',
        'value': {
            'val': 0
        }
    },
    'distance': {
        'type': 'number',
        'value': {
            'val': 0
        }
    },
    "refRobot": {
        "type": "Relationship",
        "value": "urn:ngsi-ld:Robot:1"
    }
}
```

## Room NGSI type

```yaml
{
    "id": "-",
    "type": entity_type,
    "floor": {
        "type": "number",
        "value": 0
    },
    "blueprint": {
        "type": "string",
        "value": ""
    },
    "dimensions": {
        "type": "vector",
        "value": {
            "width": 20,
            "height": 20,
            "z": 4.5,
            "resolution": 0.1
        }
    },
    "annotations": {
        "type": "list",
        "value": []
    },
    "groundType": {
        "type": "string",
        "value": ""
    },
    "origin": {
        "type": "vector",
        "value": {
            "x": 25,
            "y": 5
        }
    },
    "refWarehouse": {
        "type": "Relationship",
        "value": "urn:ngsi-ld:Warehouse:1"
    }
}
```

## Rack NGSI type

```yaml
{
    "id": "-",
    "type": entity_type,
    "maxPayload": {
        "type": "number",
        "value": 300
    },
    "dimensions": {
        "type": "vector",
        "value": {
            "length": 10,
            "width": 1,
            "height": 4,
            "orientation": 0.78539
        }
    },
    "origin": {
        "type": "vector",
        "value": {
            "x": 2,
            "y": 10
        }
    },
    "refRoom": {
        "type": "Relationship",
        "value": "urn:ngsi-ld:Room:1"
    }
}
```

## Shelf NGSI type

```yaml
{
    "id": "-",
    "type": entity_type,

    "altitude": {
        "type": "number",
        "value": 2.5
    },
    "surfaceNature": {
        "type": "string",
        "value": ""
    },
    "refRack": {
        "type": "Relationship",
        "value": "urn:ngsi-ld:Rack:1"
    }
}
```

## Slot NGSI type

```yaml
{
    "id": "-",
    "type": entity_type,
    "width": {
        "type": "number",
        "value": 0.5
    },
    "originx": {
        "type": "number",
        "value": 4
    },
    "refShelf": {
        "type": "Relationship",
        "value": "urn:ngsi-ld:Shelf:1"
    }
}
```

## Pallet NGSI type

```yaml
{
    "id": "-",
    "type": entity_type,
    "dimensions": {
        "type": "vector",
        "value": {
            "length": 0,
            "width": 0,
            "height": 0
        }
    },
    "barcode": {
        "type": "string",
        "value": {
            "val": "xxx"
        }
    },
    "material": {
        "type": "string",
        "value": ""
    },
    "fragile": {
        "type": "bool",
        "value": "false"
    },
    "refSlot": {
        "type": "Relationship",
        "value": "urn:ngsi-ld:Slot:1"
    }
}
```

## Parcel NGSI type

```yaml
{
    "id": "-",
    "type": entity_type,
    "dimensions": {
        "type": "vector",
        "value": {
            "length": 0.2,
            "width": 0.2,
            "height": 0.2
        }
    },
    "sku": {
        "type": "string",
        "value": "xxx"
    },
    "manufacturer": {
        "type": "string",
        "value": ""
    },
    "manufDate": {
        "type": "Date",
        "value": ""
    },
    "content": {
        "type": "string",
        "value": ""
    },
    "mass": {
        "type": "number",
        "value": 1
    },
    "price": {
        "type": "number",
        "value": 0
    },
    "fragile": {
        "type": "bool",
        "value": "true"
    },
    "itemQuantity": {
        "type": "number",
        "value": 10
    },
    "refPallet": {
        "type": "Relationship",
        "value": "urn:ngsi-ld:Pallet:1"
    }
}
```

---

# Robot(s) setup

In order to integrate a robotic platform to the NOOS-Open infrastructure, the robot must be ROS2-enabled, at least supporting the ROS2 Navigation stack. Furthermore, its OS must be unix-based, in order to be able to execute Python v3, as well as the [Commlib-py](https://pypi.org/project/commlib-py/) library, which acts as the communication channel between the Robot and NOOS-Open.

## Bootstrapping software

For easier integration, NOOS-Open provides a bootstrapping ROS2 package that can be easily modified for any ROS2-supporting robot and allow for quick and easy integration with the NOOS-Open infrastructure. This package is located [here](https://github.com/ortelio/Noos-Open/tree/main/basic_mobile_robot).

Below follows the instructions to execute it:

**Prerequisites**
1. Ubuntu Linux 20.04 
2. ROS 2 Foxy Fitzroy installed
3. Gazebo installed
4. Commlib-py installed 
5. Ros 2 workspace

**Instructions**
1. Copy `basic_mobile_robot` folder inside `ros2_workspace/src`
2. Build the package
```
cd ~/ros2_workspace

colcon build
```
3. Open ~/.bashrc and add the following lines to the end
```
source ~/ros2_workspace/install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_workspace/src/basic_mobile_robot/models/
```
4. Open a terminal and run the launch.file. Gazebo and RVIZ will open with Navigation2
```
cd ros2_workspace/src/basic_mobile_robot

ros2 launch basic_mobile_robot basic_mobile_bot.launch.py 
```
5. In another terminal run the script file
```
cd ros2_workspace/install/basic_mobile_robot/lib/basic_mobile_robot/

ros2 run basic_mobile_robot py_node.py

```

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


