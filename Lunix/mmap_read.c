#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <inttypes.h>

#define LUNIX_CHRDEV_BUFSZ      20      /* Buffer size used to hold textual info */

//directly from lunix.h
struct lunix_msr_data_struct {
  uint32_t magic;
	uint32_t last_update;
	uint32_t values[];
};

int main(int argc, char *argv[]){
  int fd;
  struct lunix_msr_data_struct *buff_a;
  unsigned int buf_timestamp, current_timestamp;
  current_timestamp = 0;

  printf("Sensor Filename:%s\n", argv[1]);
  if (argc != 2){
    perror("Usage: ./mmap_read /dev/lunixXX-YYYY");
    exit(1);
  }

  fd = open(argv[1], O_RDONLY);
  if (fd < 0){
    perror("Opening Sensor File failed");
    exit(2);
  }
  //mmap VA address
  buff_a = mmap(NULL, LUNIX_CHRDEV_BUFSZ, PROT_READ, MAP_PRIVATE, fd, 0);
  if (buff_a == MAP_FAILED){
    perror("Memory mapping failed");
    return 1;
  }
  //Synexhs leitourgia :)
  while(1) {
    buf_timestamp = buff_a->last_update;
    if(buf_timestamp != current_timestamp){
      printf("Current Sensor Value: %u\n", buff_a->values[0]);
    }
  }
  close(fd);
  munmap(buff_a, LUNIX_CHRDEV_BUFSZ);
  }
