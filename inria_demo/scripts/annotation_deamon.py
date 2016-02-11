#!/usr/bin/env python
import roslib
import rospy
import md5
from std_msgs.msg import String
import os

def file_checksum(file):
    m = md5.md5(open(file).read())
    return m.hexdigest()

def file_change(file, checksum):
    if checksum == None:
        return True
    else:
        return checksum != file_checksum(file)

def get_annotation(file):
    content = None
    with open(file, "r") as f:
        content = f.read()
    return content

def get_path():
    return os.path.abspath(os.path.dirname(__file__))

if __name__ == '__main__':
    file = get_path() + "/../web/bin/htdocs/config/annotations.json"
    print file
    checksum = None
    pub = rospy.Publisher('/robotcmd/annotations', String, latch=True)
    rospy.init_node('annotation_deamon', anonymous=True)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if(os.path.isfile(file) == False):
            continue
        if( file_change(file, checksum) == False):
            continue
        print "Annotation changed"
        checksum = file_checksum(file)
        annot = get_annotation(file).replace("\r\n","").replace("\t","")
        pub.publish(annot)
        r.sleep()