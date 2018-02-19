/**
Copyright (c) 2017 Xuan Sang LE <xsang.le@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
**/
#ifndef UTIL_H
#define UTIL_H
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netdb.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>

#include "data_portal.h"

#ifndef MLOG
#define MLOG printf
#endif
#define MAX_BUFF 512
#define TIME_OUT_S 3
#define TIME_OUT_U 300000
// four bytes magic header
#define  MAGIC_HEADER 0xAB2F374A
#define  PRAGMENT_SIZE 1024//65500
//#define  MAX_DGRAM_DATA 65500
struct inet_id_ {
    struct in_addr ip;
    struct in_addr netmask;
    struct in_addr  broadcast;
    char* hostname;
    unsigned char mac[6];
};

struct beacon_t {
    struct in_addr ip;
    char* hostname;
    int port;
    int status;
};
struct inet_id_ read_inet_id(const char* );
//void notify(int,int,const char*);
int send_beacon(int,const char*,int);
int bind_udp_socket(int);
struct beacon_t sniff_beacon(int, struct inet_id_);
struct portal_data_t udp_portal_checkin(int, struct inet_id_ id);
int upd_data_broadcast(int port, const char* iface, struct portal_data_t pdata);
#endif 