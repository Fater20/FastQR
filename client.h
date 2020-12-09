#include<stdio.h>  
#include<stdlib.h>  
#include<string.h>  
#include<errno.h>  
#include<sys/types.h>  
#include<sys/socket.h>  
#include<netinet/in.h>  
#include<arpa/inet.h>

#define RECV_SIZE 4096

int client_start(in_port_t port,char * ip);
int client_stop();
int client_send(char * buffer,int length);
int client_recv();