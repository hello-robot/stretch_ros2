#!/bin/bash

if [ -z "$1" ] || [ -z "$2" ]
then
    printf "Usage:\n build_docker.sh <name_of_the_tag> <path_to_dockerfile>\n"
else
    docker build --tag "$1" - < "$2"
fi

