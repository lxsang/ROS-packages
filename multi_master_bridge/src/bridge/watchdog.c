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
#include "watchdog.h"

struct inet_id_ read_inet_id(const char* inf)
{
    int fd;
    struct ifreq ifr;
    struct inet_id_ id;
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    //Type of address to retrieve - IPv4 IP address
    ifr.ifr_addr.sa_family = AF_INET;
 
    //Copy the interface name in the ifreq structure
    strncpy(ifr.ifr_name , inf , IFNAMSIZ-1);
     
    //get the ip address
    if(ioctl(fd, SIOCGIFADDR, &ifr) == 0)
        id.ip = ((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr;
     
    //get the netmask ip
    if(ioctl(fd, SIOCGIFNETMASK, &ifr) == 0)
        id.netmask = ((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr;
    
    //calculate bc address
    id.broadcast.s_addr = id.ip.s_addr | ~ id.netmask.s_addr;
    
    // get mac address  
    if (ioctl(fd, SIOCGIFHWADDR, &ifr) == 0) 
        memcpy(id.mac, ifr.ifr_hwaddr.sa_data, 6);

    char hostname[MAX_BUFF];
    hostname[MAX_BUFF-1] = '\0';
    gethostname(hostname, MAX_BUFF);
    if(strlen(hostname) > 0)
        id.hostname = strdup(hostname);
    else
        id.hostname = "unknown";

    /*MLOG("%s \n",inet_ntoa(id.ip));
    MLOG("%s \n",inet_ntoa(id.netmask));
   
    MLOG("%s \n",inet_ntoa(id.broadcast));
     printf("hostname: [%s]",hostname);
    int i=0;
    for(i;i<6;i++)
        MLOG("%.2X ", id.mac[i]);
    MLOG("\n");*/
    
    close(fd);
    return id;
}

/*void notify(int interval,int port, const char* iface)
{
    // this should be in a thread
    while(1)
    {
        MLOG("Sending beacon...\n");
        send_beacon(port,iface);
        nanosleep((const struct timespec[]){{0, (long)interval*1000000L}}, NULL);
    }
}*/

int send_beacon(int port, const char* iface, int listen_port)
{
    int sockfd; 
    struct sockaddr_in their_addr; // connector's address information
    int numbytes = 0; 
    int totalbytes;
    int broadcast = 1;
    int hostname_size;
    int magic = MAGIC_HEADER;
    char buffer[MAX_BUFF];
    struct inet_id_ id = read_inet_id(iface);
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        perror("socket");
        return 0;
    }

    // this call is what allows broadcast packets to be sent:
    if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast,
        sizeof broadcast) == -1) {
        close(sockfd);
        perror("setsockopt (SO_BROADCAST)");
        return 0;
    }

    their_addr.sin_family = AF_INET;     // host byte order
    their_addr.sin_port = htons(port); // short, network byte order
    their_addr.sin_addr = id.broadcast;
    memset(their_addr.sin_zero, '\0', sizeof their_addr.sin_zero);

    hostname_size = strlen(id.hostname);
    totalbytes = 12 + hostname_size;
    // build up the beacon
    memcpy(buffer,&magic,sizeof(int));
    memcpy(buffer+sizeof(int),&hostname_size,sizeof(int));
    memcpy(buffer+2*sizeof(int),id.hostname,hostname_size);
    memcpy(buffer+2*sizeof(int)+hostname_size,&listen_port,sizeof(int));
    
    // send out the beacon
    if ((numbytes=sendto(sockfd, buffer, totalbytes, 0,
             (struct sockaddr *)&their_addr, sizeof their_addr)) == -1) {
        perror("sendto");
        close(sockfd);
        return 0;
    }
    close(sockfd);

    //MLOG("sent %d bytes to %s\n", numbytes,inet_ntoa(their_addr.sin_addr));
    if(numbytes != totalbytes)
        return 0;
    //MLOG("Success!");
    return 1;
}
int bind_udp_socket(int port)
{
    int sockfd;
	struct addrinfo hints, *servinfo, *p;
	int rv;
	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC; // set to AF_INET to force IPv4
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_flags = AI_PASSIVE; // use my IP
    char port_a[10];
    snprintf(port_a, 10,"%d",port);
	if ((rv = getaddrinfo(NULL, port_a, &hints, &servinfo)) != 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
		return -1;
	}
	// loop through all the results and bind to the first we can
	for(p = servinfo; p != NULL; p = p->ai_next) {
		if ((sockfd = socket(p->ai_family, p->ai_socktype,
				p->ai_protocol)) == -1) {
			perror("listener: socket");
			continue;
		}

		if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
			close(sockfd);
			perror("listener: bind");
			continue;
		}

		break;
	}

	if (p == NULL) {
		fprintf(stderr, "listener: failed to bind socket\n");
        return -1;
	}
	freeaddrinfo(servinfo);
    // wait for read in 300ms
    struct timeval read_timeout;
    read_timeout.tv_sec = TIME_OUT_S;
    read_timeout.tv_usec = 0;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof read_timeout);
    
    return sockfd;
}

static int udp_broadcast_dgram(int port, struct inet_id_ id, uint8_t*gram,int size)
{
    int sockfd; 
    struct sockaddr_in their_addr; // connector's address information
    int numbytes = 0; 
    int broadcast = 1;
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        perror("socket");
        return 0;
    }

    // this call is what allows broadcast packets to be sent:
    if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast,
        sizeof broadcast) == -1) {
        close(sockfd);
        perror("setsockopt (SO_BROADCAST)");
        return 0;
    }

    their_addr.sin_family = AF_INET;     // host byte order
    their_addr.sin_port = htons(port); // short, network byte order
    their_addr.sin_addr = id.broadcast;
    memset(their_addr.sin_zero, '\0', sizeof their_addr.sin_zero);

    // ok send out the buffer
    numbytes = sendto(sockfd, gram, size, 0,(struct sockaddr *)&their_addr, sizeof their_addr);
    
    close(sockfd);
    return numbytes;
}
/*
int upd_data_broadcast(int port, const char* iface, struct portal_data_t pdata)
{
    
    int numbytes = 0; 
    int total_size = 5*sizeof(int) + strlen(pdata.publish_to) + pdata.size;
    int v = MAGIC_HEADER;
    struct inet_id_ id = read_inet_id(iface);
   
    //allocate sending buffer
    uint8_t * buffer = (uint8_t*) malloc(total_size);
    memcpy(buffer,&v, sizeof(int));
    v = total_size - 8;
    MLOG("data size %d\n",v);
    memcpy(buffer+ sizeof(int),&v,sizeof(int));
    memcpy(buffer+2*sizeof(int),&pdata.hash,sizeof(int));
     v = strlen(pdata.publish_to);
    memcpy(buffer+3*sizeof(int),&v,sizeof(int));
    memcpy(buffer+4*sizeof(int),pdata.publish_to,v);
    memcpy(buffer+4*sizeof(int)+v,&pdata.size, sizeof(int));
    memcpy(buffer+5*sizeof(int)+v,pdata.data, pdata.size);
    //MLOG("sent %d\n",v);

    int chunk = 0;
    int pragment = total_size < MAX_DGRAM_DATA?total_size:MAX_DGRAM_DATA;
    numbytes =0;
    while((chunk=udp_broadcast_dgram(port,id,buffer+numbytes,pragment)) > 0 && numbytes != total_size)
    {
        if(chunk != pragment) goto fail;
        numbytes+= chunk;
        if(total_size - numbytes < pragment) 
            pragment = total_size - numbytes;
        
    }
    if(numbytes != total_size) goto fail;
    if(buffer) free(buffer);
    MLOG("sent %d\n",numbytes);
    return 0;

    fail:
        if(buffer) free(buffer);
        MLOG("Cannot broadcast data %d \n", numbytes);
        return -1;
}
*/

int upd_data_broadcast(int port, const char* iface, struct portal_data_t pdata)
{
    int sockfd; 
    struct sockaddr_in their_addr; // connector's address information
    int numbytes = 0; 
    int broadcast = 1;
    int total_size = 0;
    //uint8_t header[8];
    struct inet_id_ id = read_inet_id(iface);
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        perror("socket");
        return 0;
    }

    // this call is what allows broadcast packets to be sent:
    if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast,
        sizeof broadcast) == -1) {
        close(sockfd);
        perror("setsockopt (SO_BROADCAST)");
        return 0;
    }

    their_addr.sin_family = AF_INET;     // host byte order
    their_addr.sin_port = htons(port); // short, network byte order
    their_addr.sin_addr = id.broadcast;
    memset(their_addr.sin_zero, '\0', sizeof their_addr.sin_zero);

    //numbytes = send(sockfd,&total_length,4,0);
        //if(numbytes != sizeof(int)) goto fail;
    total_size = 5*sizeof(int) + strlen(pdata.publish_to) + pdata.size;
    int v = MAGIC_HEADER;
    
     // send header
    //numbytes=sendto(sockfd, header, 8, 0,(struct sockaddr *)&their_addr, sizeof their_addr);
    //if(numbytes != 8) goto fail;
   
    //allocate sending buffer
    uint8_t * buffer = (uint8_t*) malloc(total_size);
    memcpy(buffer,&v, sizeof(int));
    v = total_size - 8;
    memcpy(buffer+sizeof(int),&v,sizeof(int));
    memcpy(buffer+2*sizeof(int),&pdata.hash,sizeof(int));
     v = strlen(pdata.publish_to);
    memcpy(buffer+3*sizeof(int),&v,sizeof(int));
    memcpy(buffer+4*sizeof(int),pdata.publish_to,v);
    memcpy(buffer+4*sizeof(int)+v,&pdata.size, sizeof(int));
    memcpy(buffer+5*sizeof(int)+v,pdata.data, pdata.size);
    //MLOG("sent %d to %\n",v);

    int chunk = 0;
    int pragment = total_size < PRAGMENT_SIZE?total_size:PRAGMENT_SIZE;
    numbytes =0;
    while((chunk=sendto(sockfd, buffer+numbytes, pragment, 0,(struct sockaddr *)&their_addr, sizeof their_addr))>0 && numbytes != total_size)
    {
        numbytes+= chunk;
        
        if(total_size - numbytes < pragment) 
            pragment = total_size - numbytes;
        //nanosleep((const struct timespec[]){{0, 30000L}}, NULL);
    }
    if(numbytes != total_size) goto fail;
    if(buffer) free(buffer);
    close(sockfd);
    MLOG("sent %d to %s\n",numbytes, inet_ntoa(id.broadcast) );
    return 0;

    fail:
        if(sockfd > 0)
            close(sockfd);
        if(buffer) free(buffer);
        MLOG("Cannot broadcast data %d \n", numbytes);
        return -1;
}

struct portal_data_t udp_portal_checkin(int sockfd, struct inet_id_ id)
{
    int numbytes;
	struct sockaddr_storage their_addr;
	//uint8_t header[8];
	socklen_t addr_len;
    struct portal_data_t pdata;
    struct in_addr from;
    pdata.status = 0;
	//MLOG("listener: waiting to recvfrom...\n");
	addr_len = sizeof their_addr;

     uint8_t* buffer = (uint8_t*) malloc(PRAGMENT_SIZE);
    numbytes = 0;
    int chunk = 0;
    uint8_t* rawdata = (uint8_t*) malloc(500*PRAGMENT_SIZE);
    int total_lenght = 500*PRAGMENT_SIZE;
    int pragment = PRAGMENT_SIZE;
    int magic = 0;
    int size;
    struct sockaddr * sa;
    while((chunk = recvfrom(sockfd, buffer ,pragment , 0, (struct sockaddr *)&their_addr, &addr_len)) > 0)
    {
        //if(chunk == -1) continue;
        /*if(magic == 0)
        {
            memcpy(&magic,buffer, sizeof(int));
            memcpy(&size,buffer+sizeof(int), sizeof(int));
            if(magic != MAGIC_HEADER) 
            {
                MLOG("MAGIC header is wrong %d %d %d\n", magic, MAGIC_HEADER,size);
                magic = 0;
                continue; //discard since it is not the data we want;
            }
            MLOG("Data size%d\n",size);
            sa = (struct sockaddr *)&their_addr;
            from = ((struct sockaddr_in*)sa)->sin_addr;
            if(from.s_addr == id.ip.s_addr)
            {
                //MLOG("Data sending by me, ignore it \n");
                return pdata;
            }
        }
        sa = (struct sockaddr *)&their_addr;
        if( ((struct sockaddr_in*)sa)->sin_addr.s_addr != from.s_addr){
            MLOG("this data is not for me\n");
            continue;
        } 
        if(numbytes + chunk > total_lenght)
        {
            total_lenght += 100*PRAGMENT_SIZE;
            rawdata = (uint8_t*)realloc(rawdata,total_lenght );
        }
        memcpy(rawdata+numbytes,buffer,chunk);*/
        numbytes += chunk;
    }
    if(buffer) free(buffer);
    if(numbytes != size + 8)
    {
        if(numbytes > 0)
            MLOG("Missing data. Received %d bytes\n", numbytes);
        //close(sockfd);
        return pdata;
    }
    buffer = rawdata+8;
     // parse data
    memcpy(&pdata.hash,buffer,sizeof(int));
    int v  ;
    memcpy(&v,buffer+sizeof(int),sizeof(int));
    pdata.publish_to =(char*) malloc(v+1);
    memcpy(pdata.publish_to,buffer+2*sizeof(int),v);
    pdata.publish_to[v] = '\0';
    memcpy(&pdata.size,buffer+2*sizeof(int)+v, sizeof(int));
    pdata.data =(uint8_t*) malloc(pdata.size);
    memcpy(pdata.data,buffer+3*sizeof(int)+v, pdata.size);
    pdata.status = 1;
    char* addr = inet_ntoa(from);
    v = strlen(addr);
    memcpy(pdata.from,addr, v);
    pdata.from[v] = '\0';
    if(rawdata) free(rawdata);
    return pdata;
}
struct beacon_t sniff_beacon(int sockfd,struct inet_id_ id) 
{
	int numbytes;
	struct sockaddr_storage their_addr;
	char buf[MAX_BUFF];
	socklen_t addr_len;
    struct beacon_t beacon;
    beacon.status = 0;
    struct sockaddr* sa;
	//MLOG("listener: waiting to recvfrom...\n");

	addr_len = sizeof their_addr;
	if ((numbytes = recvfrom(sockfd, buf, MAX_BUFF , 0, (struct sockaddr *)&their_addr, &addr_len)) == -1) {
        //perror("recvfrom");
        return beacon;
	}
    sa = (struct sockaddr *)&their_addr;
    if(((struct sockaddr_in*)sa)->sin_addr.s_addr == id.ip.s_addr)
    {
        //MLOG("Beacon sending by me, ignore it \n");
        return beacon;
    }
    int* v = (int*)buf;
    // read host name
    if(*v == MAGIC_HEADER && numbytes > 12)
    {
        sa = (struct sockaddr *)&their_addr;
        if (sa->sa_family == AF_INET) {
		    beacon.ip =((struct sockaddr_in*)sa)->sin_addr;

	    }
        //IPV6
	    //return &(((struct sockaddr_in6*)sa)->sin6_addr);
         // read host name 
        v = (int*)(buf+sizeof(int));
        beacon.hostname = (char*)malloc(*v+1);
        beacon.hostname[*v] = '\0';
        memcpy(beacon.hostname,buf+8,*v);
        v = (int*)(buf+2*sizeof(int)+*v);
        beacon.port = *v;
        beacon.status = 1;
        return beacon;
    }
}