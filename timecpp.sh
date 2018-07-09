#!/bin/bash


VAR=$((time /usr/bin/clang $*) 2>&1)


echo -e "time cmd: \n\n $* \n\n time run: \n\n $VAR  done"





