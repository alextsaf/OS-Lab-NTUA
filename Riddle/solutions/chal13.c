#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>


int main(int argc, char* argv[]) {

  int fd13;
  //13. Solution
  fd13 = open(".hello_there", O_RDWR);

  ftruncate(fd13,32768);

}
