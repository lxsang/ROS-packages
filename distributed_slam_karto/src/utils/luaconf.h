#ifndef LUACONF_H
#define LUACONF_H
#ifdef __cplusplus
extern "C"
{
#endif
#include "lua-5.3.4/lua.h"
#include "lua-5.3.4/lauxlib.h"
#include "lua-5.3.4/lualib.h"
#ifdef __cplusplus
}
#endif
#include <map>
#include <string>
#include <string.h>
#include "luaconf.h"

#define CONFV_NUM 0
#define CONFV_STR 1
#define CONFV_BOOL 2
#define CONFV_MAP 3

namespace dslam{
    class ConfValue;
    class Configuration;
    class Configuration{

        public:
            Configuration();
            //~Configuration(){ if(_config) delete _config ;};
            Configuration(std::string file);
            Configuration(std::string file, Configuration& cnf);
            void load(std::string file);
            void load(std::string file, Configuration&);
            template <typename T>
            T get(std::string key, T def);
            void set(std::map<std::string,ConfValue>* m) { _config = m; }
            std::map<std::string,ConfValue>* get() { return _config; }
        private:
            std::map<std::string,ConfValue>* _config;
            std::map<std::string,ConfValue>* iterateLTable(lua_State *L, int index);
            void toLTable(lua_State* L);
    };
    class ConfValue {
        public:
            ConfValue(){type=-1;};
            ConfValue(std::string);
            ConfValue(double);
            ConfValue(bool);
            ConfValue(std::map<std::string, ConfValue>*);
            ConfValue(Configuration);
            ~ConfValue();
            void get(std::string &);
            void get(double &);
            void get(bool &);
            void get(Configuration&);
            
            void set(std::string);
            void set(Configuration);
            void set(double);
            void set(bool);
            void set(std::map<std::string, ConfValue>*);
            std::string asString();
            int getType(){return type;};
        private:
            int type;
            union{
                double d;
                bool b;
                char* s;
                std::map<std::string, ConfValue>* m;
            } _value;

    };

    template <typename T>
    T Configuration::get(std::string key, T def)
    {
        T value;
        if ( _config->find(key) == _config->end() ) {
            // not found
            (*_config)[key] = ConfValue(def);
            printf("===>WARNING: Configuration %s not found. Set to  %s\n", key.c_str(), (*_config)[key].asString().c_str() );
        }
        (*_config)[key].get(value);
        return value;
    }
}

#endif