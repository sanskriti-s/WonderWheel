#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME :=

include $(IDF_PATH)/make/project.mk


cmake_minimum_required(VERSION 3.5)
include(C:\Users\sana\esp\esp-idf\tools\cmake\project.cmake)
project(Main)

CC = gcc
CFLAGS  = -g -Wall

default: count

main:  adc.o motorcontroller.o
	$(CC) $(CFLAGS) -o main adc.o motorcontroller.o

adc.o:  adc.c adc.h adc.h 
	$(CC) $(CFLAGS) -c adc.c

motorcontroller.o:  motorcontroller.c motorcontroller.h 
	$(CC) $(CFLAGS) -c motorcontroller.c

clean: 
	$(RM) count *.o *~
