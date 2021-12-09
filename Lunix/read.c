#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <inttypes.h>
#include <sys/wait.h>


int main(int argc, char* argv[]){

  pid_t pid;
  int fd;
  char buff[20];

  pid = fork();

  fd = open(argv[1], O_RDONLY);
  if (fd < 0){
    perror("Error while opening");
    exit(1);
  }
  if (pid == 0){
    read(argv[1],buff,20);
    printf("Child read: %s",buff);

  }
  else {
    read(argv[1],buff,20);
    printf("Parent read: %s,buff");
  }

close(fd);

}
