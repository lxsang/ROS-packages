## Multi-masters Bridge
This package defines base protocol for multi-masters communication (via a TCP/UDP link - UDP is a work in progress ). The idea is that each robot runs its own master, and these masters are able to automatically discover and communicate with each other for data exchange. This is achieved based on three main nodes defined by the protocol
- **beacon** : a robot will periodically broadcast its identity message to the network to identify itself.
- **robot_discover** : this node will listen to the broadcast address and discover all robots available on the network. It will then publish a topic */new_robot* which contain a list of available robots.
- **portal** : is used mainly for inter-masters data exchange. This node wait for a connection from another master. When a connection is etablished, the node receives raw data from the sender and convert it to ROS message then publish the message to a dedicated topic.

To exchange a ROS message one must define a helper to guild the protocol howto serialize ROS message to raw data (for sending) or deserialize raw data to ROS message (for receiving).

### Define a helper for a ROS message

A bash script (*helper_gen.sh*) is available in the **src/helpers** folder allowing to generate the skeleton helper class for a ROS message. For example, if one want to define a helper for *geometry_msgs/Point*, just execute :

```h
  cd src/helpers
  ./helper_gen.sh "geometry_msgs/Point"
```
Two files: *PointHelper.h* and *PointHelper.cpp* will then be generated. In the generated .cpp file, two methods need to be implemeted:

```c++
#include "PointHelper.h"

int PointHelper::hash()
{
    return _H("geometry_msgs/Point");
}
void PointHelper::rawToRosMsg(uint8_t* data)
{
    geometry_msgs::Point* ptr = new geometry_msgs::Point;
    //conversion code goes here
    _rawsize = 0;
    _msg = (void*) ptr;
}
int PointHelper::rosMsgToRaw(uint8_t** data)
{
    
    geometry_msgs::Point* ptr = (geometry_msgs::Point*)_msg;
     int len=0;
    //conversion code goes here
    return len;
}
```

The **rosMsgToRaw** method defines the serialization process of a ROS message (geometry_msgs::Point), this defines how to convert a ROS message to a bytes array. The **rawToRosMsg** method defines the inverse process of **rosMsgToRaw**.

For example implementations, please take a look at the predefined hepers in the **src/helpers** folder. Note that a helper can be used inside another helpers.

After the helper is defined, one need to register it to the protocol by adding it to the consuming class. This is done by modifying the *src/helper/DataConsumer.cpp*, change method **DataConsumer::consume** as follow:

```c++
...
if(pd->hash == StringHelper::hash())
        INPUT<std_msgs::String,StringHelper>(pub,pd);
.... 
// REGISTER the PointHelper
else if(pd->hash == PointHelper::hash())
    INPUT<geometry_msgs::Point,PointHelper>(pub,pd);
....
else
    ROS_INFO("Cannot find data handle for hash %d", pd->hash);
```
The protocol use a hash value to identify a correct helper for each exchanged data. Therefore, this step is neccessary to register the helper to the protocol.

### Sending ROS message to other master

It up to the users to define when to send a ROS message to other master, this can be performed inside a topic, or a service, etc. The only requirement is that the helper for the target ROS message must be known by the protocol. This snippet shows how to send a *geometry_msgs::Point* message to other master:

```c++
geometry_msgs::Point data;
... // some processing on data variable

//define a helper to consume the message
PointHelper hp;
// current master's ip
string my_ip = "10.1.20.19"; 
// consume the message
hp.consume((void*)&data);
// prepare raw data to be sent
struct portal_data_t d = hp.getPortalDataFor(my_ip.c_str());
// the topic will be published on received master
d.publish_to = "/dest_point";
// the hash to identify the helper
d.hash = PointHelper::hash();
/*
  send the raw data to a master
  first two params is the destination ip and port
*/
teleport_raw_data( "10.1.20.100",9191,d);
```
Note that the destination master (ip, port, etc.) can be fetched from the */new_robot* topic published by the **robot_discover** node. 

For a complete example, please take a look at the *src/map_feeding.cpp* node example to see how to exchange a complex ROS message between masters.

### Receiving ROS message from other master

Basically, when the **portal** node receive data from other master, it will deserialize received data and publish a topic of the corresponding ROS message on the receiver master (*/dest_point* topic in the previous example). User just need to subscribe to this topic to fetch the message.
