#!/usr/bin/env python

import sys
import csv
import collections
import pymongo
from pymongo import MongoClient
from pymongo.errors import ConnectionFailure

def write_bulk_to_mongodb(client, db_name, col_name, labels, row_list):
    label_length = len(labels)

    db = getattr(client, db_name)
        
    table_list = []
#    insert_num = 0
    for row in row_list:
        if(len(row) != label_length):
            continue

        doc = collections.OrderedDict() 
        for l in range(0, label_length):
            doc[labels[l]] = row[l]
        
        table_list.append(doc)

    getattr(db, col_name).insert_many(table_list)


def write_one_to_mongodb(client, db_name, col_name, labels, row_list):
    label_length = len(labels)
    db = client.kreathon

    insert_num = 0
    for row in row_list:
        if(len(row) != label_length):
            continue

        doc = collections.OrderedDict() 
        for l in range(0, label_length):
            doc[labels[l]] = row[l]
        
        db.delete_me.insert_one(doc)
        insert_num += 1
        if(insert_num > 0):
            break


def read_data_from_csv(csv_file):
    print("read data from " + csv_file)
    file_p = open(csv_file, "r")
    csv_reader = csv.reader(file_p, delimiter=",") 
    row_list = list(csv_reader)
    file_p.close()

    return row_list


if __name__ == '__main__':
    print('import csv into mongo database')
    csv_file = ""
    db_name = ""
    col_name = "" 
    if(len(sys.argv) > 3):
        csv_file = sys.argv[1]
        db_name = sys.argv[2]
        col_name = sys.argv[3]
    else:
        print('please provide a csv file')
        sys.exit(1)

    
    
    client = MongoClient('mongodb://192.168.2.117:27017')
    # check if the server is avaiable
    try:
        client.admin.command('ismaster')
    except ConnectionFailure:
        print("mongodb not avaiable")
        sys.exit(1)

    row_list = read_data_from_csv(csv_file)

    if(len(row_list) > 1):
        # parse labels 
        labels = list(row_list[0])
    else:
        print("csv file is empty")
        sys.exit(1)

    del row_list[0]

    write_bulk_to_mongodb(client, db_name, col_name, labels, row_list)
