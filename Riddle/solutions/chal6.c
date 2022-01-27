#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>


int main(int argc, char* argv[]) {
  
  int Pipe1[2], Pipe2[2];

  //6. Solution
	pipe(Pipe1);
	pipe(Pipe2);
	dup2(Pipe1[0],33);
	dup2(Pipe1[1],34);
	dup2(Pipe2[0],53);
	dup2(Pipe2[1],54);
}
