#! /bin/bash

rm -fr ./cpp/*

protoc -I=. --cpp_out=./cpp/ *.proto

rm -fr ../include/*.h
mv ./cpp/*.h ../include/
