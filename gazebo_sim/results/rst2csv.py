#! /usr/bin/python
import sys
import os
import ntpath

f = sys.argv[1]
name = ntpath.basename(f)
dir =  os.path.dirname(f)

# read the file and parse line by line
with open(f) as fp:  
    line = fp.readline()
    cnt = 0
    with open( f + ".csv", 'a') as csv_file:
        while line:
            if cnt % 2 != 0:
                # paser the line
                l = line.split()
                times = l[0].split(":")
                seconds = float(times[0])*60*60 + float(times[1])*60 + float(times[2])
                print seconds, l[1], l[2]
                csv_file.write(str(seconds) + ";" + l[1] + ";" + l[2] + "\n")
            line = fp.readline()
            cnt += 1