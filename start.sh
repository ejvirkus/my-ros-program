#! /bin/bash

echo "1 = Tegutse"
echo "2 = Joystick"
echo "Milline valik? [1,2]"

read input

clear

if [[ $input = "1" ]]
then
  echo "BUILD and RUN starting!!!"
  (cd ros_program/; bash tegutse.sh)
elif [[ $input = "2" ]]
then
  echo "Joystick starting!!!"
  (cd ros_program/; bash joystick.sh)
fi