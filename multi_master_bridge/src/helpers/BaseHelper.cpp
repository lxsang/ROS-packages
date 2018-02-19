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
#include "BaseHelper.h"

BaseHelper::~BaseHelper()
{
    //if(_msg)
    //    free(_msg);
}

portal_data_t BaseHelper::getPortalDataFor(const char* s)
{
    portal_data_t d;
    int sz = strlen(s);
    memcpy(d.from,s,sz);
    d.from[sz] = '\0';
    d.size = raw(&(d.data));
    return d;
}
void* BaseHelper::msg()
{
    return _msg;
}
int BaseHelper::raw(uint8_t**data)
{
    if(_msg)
        return this->rosMsgToRaw(data);
    return -1;
}

