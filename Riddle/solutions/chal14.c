#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>


int main(int argc, char* argv[]) {

  pid_t pid, pid2;
  //14. Solution
  while(pid != 32767){
    pid = fork();
    if (pid < 0){
      exit(-1);
    }
    if (pid == 0) {
      pid2 = getpid();
      if(pid2 == 32767){
        printf("%d\n",pid2);
        execve("riddle",NULL,NULL);
      }
      exit(0);
    }
  }
}
