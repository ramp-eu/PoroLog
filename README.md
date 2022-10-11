# Noos-Open
Warehouse automation with modular (or other) robots

> Blah blah here, regarding the general info of Noos-Open

# Deployment of Noos-Open cloud infrastructure

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

## To be added
- Security on orion context broker via keyrock
- Mosquitto certificates

[docker_tutorial]: https://docs.docker.com/engine/install/ubuntu/
[docker_compose_tutorial]: < https://docs.docker.com/compose/install/>

# Fiware & MQTT broker accompanying files

- [Commlib-py](https://pypi.org/project/commlib-py/): Robot communication controller, i.e. the software to dispatch information to Fiware broker via MQTT, using Python. This component is commlib-py and is currently open-source
- [Fiware entities JSON](https://github.com/ortelio/Noos-Open/blob/main/fiware-entities.tar.xz): Here you can find a collection of JSON-formatted files that contain the Fiware NGSI types, suitable for logistics applications
- [Hoppscotch Fiware-related REST calls](https://github.com/ortelio/Noos-Open/blob/main/hoppscotch_calls.zip): REST collections for invoking and updating all Fiware entities. Hoppscotch is the open-source alternative of Postman, alleviating its restrictions regarding membership
- [Script](https://github.com/ortelio/Noos-Open/blob/main/fiware_bootstrapping.py): Python script using [REST-ee-Fi](https://github.com/robotics-4-all/fiware-ngsi-api), via which an initial insertion of mock data to the respective Fiware entities is performed, so as for the Hoppscotch calls to operate
- [Script](https://github.com/ortelio/Noos-Open/blob/main/fiware_parcel_2d_transformation.py): Python script that takes as input a Parcel’s ID and retrieves its x,y coordinates in the absolute coordinate frame (warehouse frame)

# Codin dashboard

Codin is not open-source but it is (and will be) free for use. The dashboards can be exported in JSON formatted files and imported by another user, making the solution easily transferable. 

The Codin dashboard created for supporting Noos-Open can be found in [this link](https://github.com/ortelio/Noos-Open/blob/main/Porolog%20showcasing.json). You can import it in [Codin](https://codin.issel.ee.auth.gr/), declare your deployment's credentials and inspect your robots!

# Robotic components

- [Robot controller (ROS2 node)](): Vassilis here
- Other algorithms?
