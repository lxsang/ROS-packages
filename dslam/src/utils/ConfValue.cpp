#include "luaconf.h"

namespace dslam{
    ConfValue::ConfValue(std::string s)
    {
        set(s);
    }
    ConfValue::ConfValue(Configuration c)
    {
        set(c);
    }
    ConfValue::ConfValue(double d)
    {
        set(d);
    }
    ConfValue::ConfValue(bool b)
    {
        set(b);
    }
    ConfValue::ConfValue(std::map<std::string, ConfValue>* m)
    {
        set(m);
    }
    ConfValue::~ConfValue()
    {
        /*switch(type)
        {
            case CONFV_STR:
                free(_value.s); break;
            case CONFV_MAP:
                //if(_value.m) delete _value.m;
                break;
            default: break;
        }*/
    }
    void ConfValue::get(std::string& s)
    {
        s = std::string(_value.s);
    }
    void ConfValue::get(double &d)
    {
        d = _value.d;
    }
    void ConfValue::get(bool & b)
    {
        b = _value.b;
    }
    void ConfValue::get(Configuration& m)
    {
        m.set(_value.m);
    }
    
    void ConfValue::set(std::string s)
    {
        _value.s = strdup(s.c_str());
        type = CONFV_STR; 
    }
    void ConfValue::set(Configuration c)
    {
        type = CONFV_MAP;
        _value.m = c.get();
    }
    void ConfValue::set(double d)
    {
        _value.d = d;
        type = CONFV_NUM;
    }
    void ConfValue::set(bool b)
    {
        _value.b = b;
        type = CONFV_BOOL;
    }
    void ConfValue::set(std::map<std::string, ConfValue>* m)
    {
        _value.m = m;
        type = CONFV_MAP;
    }
        
}