#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>


int main(int argc, char* argv[]) {

  char* bfxx[10] = {"bf00","bf01","bf02","bf03","bf04","bf05","bf06","bf07","bf08","bf09"};
  int fd[10];
  //8.Solution
  for (i = 0; i < 10; i++){
		fd[i] = open(bfxx[i], O_RDWR|O_CREAT);
		lseek(fd[i], 1073741824, SEEK_SET);
		write(fd[i], "hihi uwu", 16);
		close(fd[i]);
	}
}
