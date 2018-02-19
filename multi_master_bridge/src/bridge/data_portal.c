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
#include "data_portal.h"
int portal_read_buf(int sock, char*buf,int size)
{
	int i = 0;
	int n;
    char c;
    int quit = 0;
	while (i < size  && quit == 0)
	{
		n = recv(sock, &c, 1, 0);
		if (n > 0)
		{
			buf[i] = c;
			i++;
		}
        else quit = 1;
	}
	return i;
}
int portal_request(const char* ip, int port)
{
	int sockfd, bytes_read; 
	struct sockaddr_in dest;
	//char buf[MAX_BUFF];
	char* request;
	// time out setting
	struct timeval timeout;      
	timeout.tv_sec =  TIME_OUT_S;
	timeout.tv_usec = 0;
	if ( (sockfd = socket(PF_INET, SOCK_STREAM, 0)) < 0 )
	{
		perror("Socket");
		return -1;
	}
	if (setsockopt (sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,sizeof(timeout)) < 0)
	    perror("setsockopt failed\n");

    if (setsockopt (sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout,sizeof(timeout)) < 0)
        perror("setsockopt failed\n");
	
    bzero(&dest, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons(port);
    dest.sin_addr.s_addr = inet_addr(ip);
    
	if ( connect(sockfd, (struct sockaddr*)&dest, sizeof(dest)) != 0 )
	{
		close(sockfd);
		perror("Connect");
		return -1;
	}
	return sockfd;
}

unsigned simple_hash(const char* key)
{
	unsigned hashval;
    for (hashval = 0; *key != '\0'; key++)
      hashval = *key + 31 * hashval;
	return hashval;
}

int teleport_raw_data(const char*ip, int port,struct portal_data_t pdata)
{
    int sockfd, numbytes;
    char buf[20];
    int v ;
    //int total_length = 3*sizeof(int) + strlen(pdata.from) + pdata.size;
	if((sockfd = portal_request(ip, port)) != -1)
	{
        
        //numbytes = send(sockfd,&total_length,4,0);
        //if(numbytes != sizeof(int)) goto fail;
        // send header
        v = MAGIC_HEADER;
       numbytes = send(sockfd,&v,sizeof(int),0);
        if(numbytes != sizeof(int)) goto fail;
        //MLOG("sent %d\n",v);
        // send iplen
        v = strlen(pdata.from);
        numbytes = send(sockfd,&v,sizeof(int),0);
        if(numbytes != sizeof(int)) goto fail;
        //MLOG("sent %d\n",v);
        // send ip
        numbytes = send(sockfd,pdata.from,v,0);
        if(numbytes != v) goto fail;
        //MLOG("sent %s\n",pdata.from);

        // send hash value
        numbytes = send(sockfd,&pdata.hash,sizeof(int),0);
        if(numbytes != sizeof(int)) goto fail;

        // send topic size
        v = strlen(pdata.publish_to);
        numbytes = send(sockfd,&v,sizeof(int),0);
        if(numbytes != sizeof(int)) goto fail;
        // send topic
        numbytes = send(sockfd,pdata.publish_to,v,0);
        if(numbytes != v) goto fail;

        // send raw size
        v = pdata.size;
        numbytes = send(sockfd,&v,sizeof(int),0);
        if(numbytes != sizeof(int)) goto fail;
        // MLOG("sent %d\n",pdata.size);
        // send raw data
        numbytes = send(sockfd,pdata.data,pdata.size,0);
        //MLOG("numbytes %d %d\n",numbytes,pdata.size);
        if(numbytes != pdata.size) goto fail;
        // MLOG("sent raw \n");
        //numbytes = send(sockfd,buf,total_length,0);
        //if(numbytes != total_length) goto fail;
        //read back status
        //numbytes = recv(sockfd,buf, 20, 0);
        numbytes = recv(sockfd,buf, 20, 0);
        MLOG("Received response of %d bytes\n",numbytes);
        memcpy(&v,buf,sizeof(int));
        if(numbytes != 4 || v != MAGIC_HEADER) goto fail;
		close(sockfd);
		return 0;
	}
    fail:
        if(sockfd > 0)
            close(sockfd);
        MLOG("Cannot send data to %s on %d \n",ip, port);
        return -1;
}

int portal_startup(unsigned * port)
{
    int sockfd = 0;
	struct sockaddr_in name;

	sockfd = socket(PF_INET , SOCK_STREAM, 0);
	if (sockfd == -1)
		return -1;
		//error_die("socket");
	memset(&name, 0, sizeof(name));
	name.sin_family = AF_INET;
	name.sin_port = htons(*port);
	name.sin_addr.s_addr = INADDR_ANY;
	if (bind(sockfd, (struct sockaddr *)&name, sizeof(name)) < 0)
        goto fail;
    if (*port == 0)  /* if dynamically allocating a port */
	{
		socklen_t namelen = sizeof(name);
		if (getsockname(sockfd, (struct sockaddr *)&name, &namelen) == -1)
			goto fail;
		*port = ntohs(name.sin_port);
    }
	if (listen(sockfd, 100) < 0)
		goto fail;
		//error_die("listen");
    return(sockfd);
    fail:
        close(sockfd);
        return -1;
}

int portal_listen(int sock) 
{
    struct sockaddr_in client_name;
    socklen_t client_name_len = sizeof(client_name);
    int client_sock;
	struct timeval timeout;      
	timeout.tv_sec =  TIME_OUT_S;
	timeout.tv_usec = 0;
    fd_set active_fd_set;
    FD_ZERO (&active_fd_set);
    FD_SET (sock, &active_fd_set);
    //read_fd_set = active_fd_set;
    if (select (FD_SETSIZE, &active_fd_set, NULL, NULL, &timeout) < 0)
    {
        perror ("select");
        return -1;
    }
    if (FD_ISSET (sock, &active_fd_set))
    {
        client_sock = accept(sock,(struct sockaddr *)&client_name,&client_name_len);
        if (client_sock == -1)
        {
            perror("Cannot accept client request\n");
            return -1;
        }
        setsockopt (client_sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,
                        sizeof(timeout));
        setsockopt (client_sock, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout,
                    sizeof(timeout));
        // call the handle to read data
        MLOG("%s", "Client accepted\n");
        
        return client_sock;
    }
    return -1;
}

void portal_serve(int sock,void (*callback)(struct portal_data_t)) 
{
   
    int client_sock;
    pthread_t newthread;
    
    while(1)
    {
        client_sock = portal_listen(sock);
        if (client_sock == -1)
        {
            return;
        }
        struct portal_callback_t callback_data;
        callback_data.client = client_sock;
        callback_data.callback = callback;
        if (pthread_create(&newthread , NULL,(void *(*)(void *))portal_checkin, (void *)&callback_data) != 0)
            perror("pthread_create");
        else
        {
            //reclaim the stack data when thread finish
            pthread_detach(newthread) ;
        }
        //portal_checkin(&callback_data);
    }
}

void portal_checkin(void* rawdata)
{
    int v = 0;
    int numbytes = 0;
    char buf[10];
   
    struct portal_data_t data;
    data.status = 0;
    struct portal_callback_t* call = (struct portal_callback_t*)rawdata;
    
    // read passport to check magic heade
    numbytes = recv(call->client,buf,4,0);
    if(numbytes != sizeof(v) && v != MAGIC_HEADER) goto fail;
    //MLOG("Magic header ok %d\n",v);
    
    // read IP size
    numbytes = recv(call->client,&v,sizeof(int),0);
    if(numbytes != sizeof(int)) goto fail;
    //MLOG("IP size ok %d\n",v);
    
    // read ip address
    numbytes = recv(call->client,data.from,v,0);
    if(numbytes != v) goto fail;
    data.from[v] = '\0';
    //MLOG("IP %s\n\n",data.from);
    
    // read hash
    numbytes = recv(call->client,&data.hash,sizeof(int),0);
    if(numbytes != sizeof(int)) goto fail;

    //topic size
     numbytes = recv(call->client,&v,sizeof(int),0);
    if(numbytes != sizeof(int)) goto fail;
    //topic data
    data.publish_to = (char*)malloc(v+1);
    numbytes = recv(call->client,data.publish_to,v,0);
    if(numbytes != v) goto fail;
    data.publish_to[v] ='\0';

    // read data size
    numbytes = recv(call->client,&data.size,sizeof(int),0);
    if(numbytes != sizeof(int)) goto fail;
    //MLOG("data size ok %d\n",data.size);
    
    // read all remain raw data
    data.data = (uint8_t*)malloc(data.size);
    if(!data.data)
    {
        MLOG("MALLOC: Cannot allocate data\n");
        goto fail;
    }
    int chunk = 0;
    numbytes = 0;
    while((chunk = recv(call->client,data.data+numbytes,data.size - numbytes,0))>0 && numbytes != data.size)
        numbytes += chunk;
    if(numbytes != data.size) goto fail;
    //data.data[v] = '\0'; 
    //MLOG("data %s\n",data.data);
    data.status = 1;
    
    //MLOG("%d bytes read. send back ack %d\n",numbytes);
    // send header
    v = MAGIC_HEADER;
    numbytes = send(call->client,&v,sizeof(int),0);
    if(numbytes != sizeof(v)) goto fail;
    close(call->client);
    call->callback(data);
    MLOG("Data processed\n");
    return;
    // send back the ack data
    fail:
        close(call->client);
        MLOG("Cannot read client data \n");
}