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
#ifndef BASE_HELPER_H
#define BASE_HELPER_H
#include <ros/ros.h>

#define _H simple_hash
#ifdef __cplusplus
extern "C"
{
#endif
#include "../bridge/data_portal.h"

#ifdef __cplusplus
}
#endif
using namespace std;
class BaseHelper{
    public:
        BaseHelper(){};
        ~BaseHelper();
        void* msg();
        int raw(uint8_t**);
        portal_data_t getPortalDataFor(const char*);
        static int hash(){};
        void consume(uint8_t*v,int s){_rawsize=s;this->rawToRosMsg(v);};
        void consume(void*v){_msg = v;};
        int rawsize(){return _rawsize;};
        template <class FT> void publish(ros::Publisher*);
    protected:
        int _rawsize;
        virtual void  rawToRosMsg(uint8_t*)=0;
        virtual int rosMsgToRaw(uint8_t**)=0;
        
        void* _msg;
};

template <class FT> void BaseHelper::publish(ros::Publisher* pub)
{
    if(_msg && pub)
        pub->publish(*((FT*)_msg));
}

#endif