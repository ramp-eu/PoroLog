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
import math
import pprint

API_KEY = "1234abcd"

def retrieve_parcel_pose(parcel_id):
    parcel_x = 0
    parcel_y = 0
    parcel_z = 0

    # Retrieve parcel
    res = ngsi.retrieve_entity(parcel_id, type = "Parcel")
    resp = json.loads(res.data)
    if "refPallet" in resp:
        print(parcel_id + " is in pallet " + resp['refPallet']["value"])

    else:
        print (parcel_id + " does not belong in a Pallet, must search for a slot")
        exit(0)

    # Retrieve pallet
    pallet_id = resp['refPallet']["value"]
    res = ngsi.retrieve_entity(pallet_id, type = "Pallet")
    resp = json.loads(res.data)
    if "refSlot" in resp:
        print(pallet_id + " is in slot " + resp['refSlot']["value"])
    else:
        print(pallet_id + " does not belong in a Slot")
        exit(0)

    # Retrieve slot
    slot_id = resp['refSlot']["value"]
    res = ngsi.retrieve_entity(slot_id, type = "Slot")
    resp = json.loads(res.data)
    if "refShelf" in resp:
        print(slot_id + " is in shelf " + resp['refShelf']["value"])
    else:
        print(slot_id + " does not belong in a Shelf")
        exit(0)

    slot_origin_x = resp['originx']["value"]
    parcel_x += slot_origin_x
    print("Parcel coordinates thus far: ", parcel_x, parcel_y, parcel_z)

    # Retrieve shelf
    shelf_id = resp['refShelf']["value"]
    res = ngsi.retrieve_entity(shelf_id, type = "Shelf")
    resp = json.loads(res.data)
    if "refRack" in resp:
        print(shelf_id + " is in shelf " + resp['refRack']["value"])
    else:
        print(shelf_id + " does not belong in a Racj")
        exit(0)

    shelf_altitude = resp['altitude']["value"]
    parcel_z += shelf_altitude
    print("Parcel coordinates thus far: ", parcel_x, parcel_y, parcel_z)
    
    # Retrieve rack
    rack_id = resp['refRack']["value"]
    res = ngsi.retrieve_entity(rack_id, type = "Rack")
    resp = json.loads(res.data)
    if "refRoom" in resp:
        print(rack_id + " is in room " + resp['refRoom']["value"])
    else:
        print(rack_id + " does not belong in a Room")
        exit(0)

    rack_orientation = resp['dimensions']['value']['orientation']
    rack_origin_x = resp['origin']['value']['x']
    rack_origin_y = resp['origin']['value']['y']

    parcel_y = math.sin(rack_orientation) * parcel_x + rack_origin_y
    parcel_x = math.cos(rack_orientation) * parcel_x + rack_origin_x
    print("Parcel coordinates thus far: ", parcel_x, parcel_y, parcel_z)

    # Retrieve room
    room_id = resp['refRoom']["value"]
    res = ngsi.retrieve_entity(room_id, type = "Room")
    resp = json.loads(res.data)
    if "refWarehouse" in resp:
        print(room_id + " is in warehouse " + resp['refWarehouse']["value"])
    else:
        print(room_id + " does not belong in a Warehouse")
        exit(0)

    room_origin_x = resp['origin']['value']['x']
    room_origin_y = resp['origin']['value']['y']

    parcel_x += room_origin_x
    parcel_y += room_origin_y
    print("Final parcel coordinates (in meters): ", parcel_x, parcel_y, parcel_z)

    # Retrieve warehouse
    warehouse_id = resp['refWarehouse']["value"]
    res = ngsi.retrieve_entity(warehouse_id, type = "Warehouse")
    resp = json.loads(res.data)
    print("Warehouse resolution is ", resp['dimensions']['value']['resolution'], " meters per pixel")
    resolution = resp['dimensions']['value']['resolution']

    print("Final parcel coordinates (in pixels): ", parcel_x/resolution, parcel_y/resolution)

    return {
        'x': parcel_x,
        'y': parcel_y,
        'z': parcel_z,
        'x_px': parcel_x/resolution,
        'y_px': parcel_y/resolution
    }


if __name__ == "__main__":
    config = NgsiConfiguration(file_path="../conf/settings.conf")
    api_client = NgsiApiClient(configuration=config)

    device_api = NgsiDevice(api_client=api_client)
    service_api = NgsiService(api_client=api_client)
    service_api.delete(api_key=API_KEY)
    ngsi = NgsiEntities(api_client)

    parcel_id = "urn:ngsi-ld:Parcel:1"
    pose = retrieve_parcel_pose(parcel_id)
    print("Function returned: ")
    pprint.pprint(pose)