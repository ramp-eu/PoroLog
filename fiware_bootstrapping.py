from urllib import response
from fiware_ngsi_api.api_client import NgsiApiClient
from fiware_ngsi_api.configuration import NgsiConfiguration
from fiware_ngsi_api.api.services import NgsiService
from fiware_ngsi_api.api.entities import NgsiEntities
from fiware_ngsi_api.models.robot_api import NgsiRobotAPI
from fiware_ngsi_api.api.devices import NgsiDevice

import os
import json
import time
import yaml

API_KEY = "1234abcd"


def checker(response):
    if not successfull_response(response.status):
        print(
            f"Error when creating entity with status: {response.status} and msg: {response.data}")
    else:
        print("Created successfully!")


def successfull_response(status_code):
    return True if (status_code >= 200 and status_code < 300) else False


def cleanup_type(ngsi_type, ngsi, device_api, service_api):
    print("<<<-------------------------------->>>")
    print("Preparing " + ngsi_type + " type")
    res = ngsi.list_entities(type=ngsi_type)
    resp = json.loads(res.data)
    print("Number of entities: " + str(len(resp)))
    for i in resp:
        print("Deleting " + i['id'])
        ngsi.remove_entity(i['id'])
    time.sleep(2)

    response = device_api.list(type=ngsi_type)
    response_msg = json.loads(response.data)

    print("Devices found: " + str(len(response_msg['devices'])))
    for i in response_msg['devices']:
        device_api.delete(id=i['device_id'])
        # TODO: DOES NOT DELETE THE DEVICES!!

    print(f"Searching for existing services of type {ngsi_type} ...")
    response = service_api.list()
    if (successfull_response(response.status)):
        response = json.loads(response.data)
        for service in response["services"]:
            if service["entity_type"] == ngsi_type:
                print(
                    f"Found {ngsi_type} service with API Key = {service['apikey']}")

                time.sleep(1)

                print("Now deleting it ...")
                response = service_api.delete(api_key=service['apikey'])
                if (not successfull_response(response.status)):
                    print(
                        f"Error occured when tried to delete {ngsi_type} service")

    response = service_api.create(
        api_key=f"{API_KEY}{ngsi_type}", type=ngsi_type)

    if (not successfull_response(response.status)):
        print(
            f"Error when creating service {ngsi_type} with msg {response.data}")
    else:
        print(f"Service {ngsi_type} created succesfully!")


if __name__ == "__main__":
    config = NgsiConfiguration(file_path="../conf/settings.conf")
    api_client = NgsiApiClient(configuration=config)

    device_api = NgsiDevice(api_client=api_client)
    service_api = NgsiService(api_client=api_client)
    ngsi = NgsiEntities(api_client)

    #---------------------- Warehouse handling --------------------------#
    entity_type = "Warehouse"
    cleanup_type(entity_type, ngsi, device_api, service_api)
    time.sleep(1)

    entity_yaml = {
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

    print("Creating Warehouse...")
    entity_yaml["id"] = f"urn:ngsi-ld:{entity_type}:{1}"
    checker(ngsi.create_entity(entity_yaml))

    time.sleep(0.5)

    #---------------------- WarehouseKPI handling --------------------------#
    entity_type = "WarehouseKPI"
    cleanup_type(entity_type, ngsi, device_api, service_api)
    time.sleep(1)

    entity_yaml = {
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

    print("Creating WarehouseKPI...")
    entity_yaml["id"] = f"urn:ngsi-ld:{entity_type}:{1}"
    checker(ngsi.create_entity(entity_yaml))
    time.sleep(0.5)

    #---------------------- Robots handling --------------------------#
    cleanup_type("Robot", ngsi, device_api, service_api)
    time.sleep(1)

    service_api.list()

    # Load robot yaml
    robot_yaml = {}
    if os.path.exists("../data-models/bootstrapping/robot1.yaml"):
        with open("../data-models/bootstrapping/robot1.yaml", "r") as stream:
            try:
                robot_yaml = yaml.safe_load(stream)['Robot']
            except Exception as e:
                print(e)
    else:
        raise IOError("Invalid robot yaml path!")

    print("Creating Robot1...")
    robot_yaml['id'] = 1
    robot_yaml['name'] = "Robot" + str(robot_yaml['id'])
    robot_yaml['refWarehouse'] = {
        "type": "Relationship",
        "value": "urn:ngsi-ld:Warehouse:1"
    }
    robot1 = NgsiRobotAPI(api_client, robot_yaml)
    time.sleep(0.5)

    print("Creating Robot2...")
    robot_yaml['id'] = 2
    robot_yaml['name'] = "Robot" + str(robot_yaml['id'])
    robot_yaml['refWarehouse'] = {
        "type": "Relationship",
        "value": "urn:ngsi-ld:Warehouse:1"
    }
    robot1 = NgsiRobotAPI(api_client, robot_yaml)
    time.sleep(0.5)

    #---------------------- Robots KPIs handling --------------------------#
    entity_type = "RobotKPI"
    cleanup_type(entity_type, ngsi, device_api, service_api)
    time.sleep(1)

    entity_yaml = {
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

    print("Creating RobotKPI1...")
    entity_yaml["id"] = f"urn:ngsi-ld:{entity_type}:{1}"
    checker(ngsi.create_entity(entity_yaml))
    time.sleep(0.5)

    #---------------------- Room  handling --------------------------#
    entity_type = "Room"
    cleanup_type(entity_type, ngsi, device_api, service_api)
    time.sleep(1)

    entity_yaml = {
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

    print("Creating Room1...")
    entity_yaml["id"] = f"urn:ngsi-ld:{entity_type}:{1}"
    checker(ngsi.create_entity(entity_yaml))
    time.sleep(0.5)

    #---------------------- Rack  handling --------------------------#
    entity_type = "Rack"
    cleanup_type("Rack", ngsi, device_api, service_api)
    time.sleep(1)

    entity_yaml = {
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

    print("Creating Rack1...")
    entity_yaml["id"] = f"urn:ngsi-ld:{entity_type}:{1}"
    checker(ngsi.create_entity(entity_yaml))
    time.sleep(0.5)

    #---------------------- Shelf  handling --------------------------#
    entity_type = "Shelf"
    cleanup_type(entity_type, ngsi, device_api, service_api)
    time.sleep(1)

    entity_yaml = {
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

    print("Creating Shelf1...")
    entity_yaml["id"] = f"urn:ngsi-ld:{entity_type}:{1}"
    checker(ngsi.create_entity(entity_yaml))
    time.sleep(0.5)

    #---------------------- Slot  handling --------------------------#
    entity_type = "Slot"
    cleanup_type(entity_type, ngsi, device_api, service_api)
    time.sleep(1)

    entity_yaml = {
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

    print("Creating Slot1...")
    entity_yaml["id"] = f"urn:ngsi-ld:{entity_type}:{1}"
    checker(ngsi.create_entity(entity_yaml))
    time.sleep(0.5)

    #---------------------- Pallet  handling --------------------------#
    entity_type = "Pallet"
    cleanup_type(entity_type, ngsi, device_api, service_api)
    time.sleep(1)

    entity_yaml = {
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

    print("Creating Pallet1...")
    entity_yaml["id"] = f"urn:ngsi-ld:{entity_type}:{1}"
    checker(ngsi.create_entity(entity_yaml))
    time.sleep(0.5)

    #---------------------- Pallet  handling --------------------------#
    entity_type = "Parcel"
    cleanup_type(entity_type, ngsi, device_api, service_api)
    time.sleep(1)

    entity_yaml = {
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

    print("Creating Parcel1...")
    entity_yaml["id"] = f"urn:ngsi-ld:{entity_type}:{1}"
    checker(ngsi.create_entity(entity_yaml))
    time.sleep(0.5)

    # to end...
    time.sleep(5)
