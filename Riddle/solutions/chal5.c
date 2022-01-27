#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>


int main(int argc, char* argv[]) {
  int fd5;
  //5. Solution
  fd5 = open("test",O_CREAT|O_RDWR);
  dup2(fd5,99);
}
