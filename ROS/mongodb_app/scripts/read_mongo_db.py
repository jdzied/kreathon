#!/usr/bin/env python

import sys
import csv
import collections
import pymongo
from pymongo import MongoClient
from pymongo.errors import ConnectionFailure
import pprint
from bson.objectid import ObjectId
import time

client = MongoClient('mongodb://192.168.2.117:27017')
# check if the server is avaiable
try:
	client.admin.command('ismaster')
except ConnectionFailure:
	print("mongodb not avaiable")
	sys.exit(1)


db = getattr(client, 'kreathon')

Wareneingang = getattr(db, 'Wareneingang')

print("Sleeping and then printing item with _id = 5b0d750737997b0f1bb892b6")
time.sleep(2)

# look for one specific element by ID
pprint.pprint(Wareneingang.find_one({"_id": ObjectId("5b0d750737997b0f1bb892b6")}))   



print("Sleeping and then printing all items on Anlieferstelle 9NE")
time.sleep(2)

# look for alle items with specific number
for item in Wareneingang.find({"Anlieferstelle": "9NE"}):
	pprint.pprint(item)