#!/bin/bash

curl -v -X POST http://localhost:5000/do -d p="[{\"y\":$2, \"x\":$1, \"index\":0, \"t\":0.0, \"name\":\"goto\"}]"
