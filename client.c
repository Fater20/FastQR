#include "client.h"
 
int    socket_fd, connect_fd;  
struct sockaddr_in server_addr; 

char recv_buffer[RECV_SIZE];

int client_start(in_port_t port,char * ip)
{
    if( (socket_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1 )
    {  
        printf("create socket error: %s(errno: %d)\n",strerror(errno),errno);  
        return 0;
    }
    memset(&server_addr, 0, sizeof(server_addr));  
    server_addr.sin_family = AF_INET;  
    if( inet_pton(AF_INET, ip, &server_addr.sin_addr) <= 0)
    {  
        printf("inet_pton error for %s\n",ip);  
        return 0;  
    }  
    server_addr.sin_port = htons(port);
    if( connect(socket_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
    {  
        printf("connect error: %s(errno: %d)\n",strerror(errno),errno);  
        return 0;
    }
    return 1;
}

int client_send(char * buffer,int length)
{
    if( send(socket_fd, buffer, length, 0) < 0)  
    {  
        printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);  
        return 0;  
    }  
}

int client_recv()
{
    ;
}