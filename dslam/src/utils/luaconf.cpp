
#include "luaconf.h"

namespace dslam
{
Configuration::Configuration()
{
    _config = new std::map<std::string, ConfValue>();
}
Configuration::Configuration(std::string file)
{
    _config = new std::map<std::string, ConfValue>();
    load(file);
}

void Configuration::load(std::string file)
{
    lua_State *L = NULL;
    L = luaL_newstate();
    if (luaL_loadfile(L, file.c_str()) || lua_pcall(L, 0, 1, 0))
    {
        printf("ERROR: cannot load configuration file. file: %s\n", lua_tostring(L, -1));
        return;
    }
    // now convert lua table to map of value
    if (L)
    {
        if (lua_istable(L, 1))
        {
            if(_config) delete _config;
            _config = iterateLTable(L,-1);
        }
        lua_close(L);
    }
}

std::map<std::string, ConfValue>* Configuration::iterateLTable(lua_State *L, int index)
{
    // Push another reference to the table on top of the stack (so we know
    // where it is, and this function can work for negative, positive and
    // pseudo indices
    std::map<std::string, ConfValue>* m  = new std::map<std::string, ConfValue>();
    lua_pushvalue(L, index);
    // stack now contains: -1 => table
    lua_pushnil(L);
    // stack now contains: -1 => nil; -2 => table
    while (lua_next(L, -2))
    {
        // stack now contains: -1 => value; -2 => key; -3 => table
        // copy the key so that lua_tostring does not modify the original
        lua_pushvalue(L, -2);
        // stack now contains: -1 => key; -2 => value; -3 => key; -4 => table
        const char *key = lua_tostring(L, -1);
        if (lua_istable(L, -2))
        {
            // the element is a table
            // create new dictionary
            std::map<std::string, ConfValue>* cm = iterateLTable(L, -2);

            (*m)[key] = ConfValue(cm);
        }
        else if(lua_isnumber(L,-2))
        {
            (*m)[key] = ConfValue((double)lua_tonumber(L,-2));
        }
        else if(lua_isboolean(L,-2))
        {
            (*m)[key] = ConfValue(lua_toboolean(L,-2) == 1?true:false);
        }
        else
        {
            // other value is converted to string
            const char *value = lua_tostring(L, -2);
            (*m)[key] = std::string(value);
        }

        // pop value + copy of key, leaving original key
        lua_pop(L, 2);
        // stack now contains: -1 => key; -2 => table
    }
    // stack now contains: -1 => table (when lua_next returns 0 it pops the key
    // but does not push anything.)
    // Pop table
    lua_pop(L, 1);
    // Stack is now the same as it was on entry to this function
    return m;
}
}