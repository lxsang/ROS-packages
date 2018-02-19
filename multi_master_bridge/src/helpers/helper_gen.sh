#!/bin/bash
IFS='/' read -r -a array <<< "$1"

pattern_0="$1"
if [ ${#array[@]} -eq 2 ]; then
    pattern_1=`echo "${array[1]}" | tr '[:lower:]' '[:upper:]'`
    pattern_2="${array[1]}Helper"
    pattern_3="${array[0]}"
    pattern_4="${array[1]}"
    
    cp -i helperh.tpl "$pattern_2.h"
    cp -i helpercpp.tpl "$pattern_2.cpp"

    arr=( "$pattern_2.h" "$pattern_2.cpp" )

    for i in "${arr[@]}"; do
        sed -i  "s@%0@$pattern_0@g" "$i"
        sed -i  "s@%1@$pattern_1@g" "$i"
        sed -i  "s@%2@$pattern_2@g" "$i"
        sed -i  "s@%3@$pattern_3@g" "$i"
        sed -i  "s@%4@$pattern_4@g" "$i"
        echo "generated $i"
    done
    sed -i '$ d' helper.h 
    echo "#include \"$pattern_2.h\"">> helper.h
    echo "#endif">> helper.h
else
    echo "Wrong message type"
fi
