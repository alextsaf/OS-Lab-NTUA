#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
//#include "lunix-chrdev.h"

int main(int argc, char *argv[]){
  ssize_t read_bytes;
  int buff[30];
  int fd;

  fd = open(argv[1], O_RDONLY);
  if (fd < 0){
    perror("Error while opening device from userspace");
    return 1;
  }


  read_bytes = read(fd,buff,1);

  while(read_bytes > 0){
    printf("%ls",buff);
    read_bytes = read(fd,buff,1);
    if (read_bytes == -1){
      perror("Error while reading buffer from userspace");
      exit(0);
  }
  }
}
