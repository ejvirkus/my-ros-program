#! /bin/bash

echo "HAKKAB TEGUTSEMA!!!"

dts devel build -f -H ejvirkus.local
dts devel run -H ejvirkus.local -- --privileged

echo "SAI TEGUTSETUD!!!"
