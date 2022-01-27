#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>


int main(int argc, char* argv[]) {

  int fd11;
  pid_t pid11;
  char buff11[100];
  
  //11. Solution
  fd11 = openat(AT_FDCWD,"secret_number", O_RDWR|O_CREAT,0600);
  pid11 = fork();
  if (pid < 0){
    perror("error while forking");
    exit(-1);
  }
  else if (pid > 0){
    sleep(10);
    read(fd11,buff11,100);
    printf("buffer content: %s\n",buff11);
    sleep(10);
  }
  else {
    execve("riddle",NULL,NULL);
  }
	}
