#include "plugin.h"

#define MAX_FILE_SIZE 524288

void init()
{
	
}
void pexit()
{
	
}
static void result(void * client, char* s)
{
    __t(client,"{ \"error\": false, \"result\":\"%s\" }", s);
}

static void error(void * client, char* s)
{
    __t(client,"{ \"error\":\"%s\" }", s);
}

void handle(void* client, const char* m, const char* rqp, dictionary rq)
{
    json(client);
    char* tmp_name = (char*)dvalue(rq, "conf.tmp");
    char* file_name = (char*)dvalue(rq, "conf.file");
    if( tmp_name && file_name)
    {
        int size = atoi((char*)dvalue(rq,"conf.size"));
        if(size > MAX_FILE_SIZE)
        {
            error(client, "File size is too big");
            return;
        }
        char* path = __s("%s/config/%s",__plugin__.htdocs,"annotations.json");
        if(rename(tmp_name, path) == -1)
        {
            error(client, "Cannot upload file");
        }
        else
        {
            result(client, "File uploaded");
        }
        free(path);
    }
    else
    {
        error(client, "Unknow uploaded file");
    }
}