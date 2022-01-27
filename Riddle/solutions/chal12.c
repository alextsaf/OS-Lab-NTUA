#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>


int main(int argc, char* argv[]) {

  int fd12;
	char temp[2];

  //12. Solution
	temp[0] = *argv[2];
	fd12 = open(argv[1],O_RDWR);
	if (!fd12) {
		perror("ekanes vlakeia");
		exit(-1);
	}
	lseek(fd12,0x6f,SEEK_SET);
	write(fd12,temp,sizeof(temp[0]));
}
