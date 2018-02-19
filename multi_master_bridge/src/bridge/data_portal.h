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
#ifndef DATA_PORTAL_H
#define DATA_PORTAL_H
#include <pthread.h>
#include "watchdog.h"


struct portal_data_t{
    char        from[20];
    unsigned    hash;
    int         size;
    char*       publish_to;
    uint8_t*    data;
    int         status;
};

struct portal_callback_t{
    int client;
    void (*callback)(struct portal_data_t);
};

int portal_request(const char*,int);
int teleport_raw_data(const char*,int,struct portal_data_t);
int portal_startup(unsigned * );
int portal_listen(int sock);
void portal_serve(int,void (*callbacl)(struct portal_data_t));
void portal_checkin(void*);
unsigned simple_hash(const char*);
#endif