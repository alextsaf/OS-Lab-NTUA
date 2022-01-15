/*
* socket-client.c
* Simple TCP/IP communication using sockets
*
* Vangelis Koukis <vkoukis@cslab.ece.ntua.gr>
*/

#include <stdio.h>
#include <errno.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <netdb.h>
#include <fcntl.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <poll.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <crypto/cryptodev.h>

#include "crypto-common.h"


#define DATA_SIZE       256
#define BLOCK_SIZE      16
#define KEY_SIZE	16  /* AES128 */

//initializations
unsigned char buf[DATA_SIZE];
struct session_op sess;
unsigned char key[KEY_SIZE] = "mpougatsiarides1";
unsigned char iv[BLOCK_SIZE] = "hsfairaponanana1";


/* Insist until all of the data has been written */
ssize_t insist_write(int fd, const void *buf, size_t cnt)
{
	ssize_t ret;
	size_t orig_cnt = cnt;

	while (cnt > 0) {
		ret = write(fd, buf, cnt);
		if (ret < 0)
		return ret;
		buf += ret;
		cnt -= ret;
	}

	return orig_cnt;
}

int encrypt_data(int cfd)
{
	int i;
	struct crypt_op cryp;
	struct {
		unsigned char 	in[DATA_SIZE],
				encrypted[DATA_SIZE];
			} data;


	memset(&cryp, 0, sizeof(cryp));
	//Initialize crypto struct with crypto session info
	cryp.ses = sess.ses;
	cryp.len = sizeof(buf);
	cryp.src = buf;
	cryp.dst = data.encrypted;
	cryp.iv = iv;
	cryp.op = COP_ENCRYPT;

	//send IO command to encrypt from buf to data.encrypted
	if (ioctl(cfd, CIOCCRYPT, &cryp)) {
		perror("Error encrypting");
		return 1;
	}
	//init empty buffer
	memset(buf, '\0', sizeof(DATA_SIZE));
	//for buffer-size fill it with encrypted data
	for (i = 0; i < sizeof(buf); i++){
		buf[i] = data.encrypted[i];
	}

	//Function returns 0 on success
	return 0;
}


int decrypt_data(int cfd)
{
	int i;
	struct crypt_op cryp;
	struct {
		unsigned char 	in[DATA_SIZE],
				decrypted[DATA_SIZE];
			} data;

	memset(&cryp, 0, sizeof(cryp));


	//Initialize crypto struct with crypto session info
	cryp.ses = sess.ses;
	cryp.len = sizeof(buf);
	cryp.src = buf;
	cryp.dst = data.decrypted;
	cryp.iv = iv;
	cryp.op = COP_DECRYPT;

	//send IO command to decrypt from buf to data.decrypted
	if (ioctl(cfd, CIOCCRYPT, &cryp)) {
		perror("Error decrypting");
		return 1;
	}
	
	//init empty buffer
	memset(buf, '\0', sizeof(DATA_SIZE));

	//for buffer-size fill it with decrypted data
	for (i = 0; i < sizeof(buf); i++) {
				buf[i] = data.decrypted[i];
	}

	//Function returns 0 on success
	return 0;
}



int main(int argc, char *argv[])
{
	int sd, port;
	ssize_t n;

	char *hostname;
	struct hostent *hp;
	struct sockaddr_in sa;

	memset (&sess, 0, sizeof(sess));
	if (argc != 3) {
		fprintf(stderr, "Usage: %s hostname port\n", argv[0]);
		exit(1);
	}
	hostname = argv[1];
	port = atoi(argv[2]); /* Needs better error checking */

	/* Create TCP/IP socket, used as main chat channel */
	if ((sd = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
		perror("socket");
		exit(1);
	}
	fprintf(stderr, "Created TCP socket\n");

	/* Look up remote hostname on DNS */
	if ( !(hp = gethostbyname(hostname))) {
		printf("DNS lookup failed for host %s\n", hostname);
		exit(1);
	}

	/* Connect to remote TCP port */
	sa.sin_family = AF_INET;
	sa.sin_port = htons(port);
	memcpy(&sa.sin_addr.s_addr, hp->h_addr, sizeof(struct in_addr));
	fprintf(stderr, "Connecting to remote host... "); fflush(stderr);
	if (connect(sd, (struct sockaddr *) &sa, sizeof(sa)) < 0) {
		perror("connect");
		exit(1);
	}
	fprintf(stderr, "Connected.\n");

	//Open Cryptodev module
	int crypto_fd = open("/dev/crypto", O_RDWR);
	if (crypto_fd < 0){
		perror("Opening cryptodev failed");
		exit(1);
	}

	//Initialize session
	sess.cipher = CRYPTO_AES_CBC;
	sess.keylen = KEY_SIZE;
	sess.key = key;

	if (ioctl(crypto_fd, CIOCGSESSION, &sess)) { //send IO command to begin session
		perror("Crypto session init failed");
		return 1;
	}

	// /* Be careful with buffer overruns, ensure NUL-termination */
	// strncpy(buf, HELLO_THERE, sizeof(buf));
	// buf[sizeof(buf) - 1] = '\0';

	// /* Say something... */
	// if (insist_write(sd, buf, sizeof(buf)) != sizeof(buf)) {
	// 	perror("write");
	// 	exit(1);
	// }
	// // fprintf(stdout, "I said:\n%s\nRemote says:\n", buf);
	// fflush(stdout);

	/*
	* Let the remote know we're not going to write anything else.
	* Try removing the shutdown() call and see what happens.
	*/
	// if (shutdown(sd, SHUT_WR) < 0) {
	// 	perror("shutdown");
	// 	exit(1);
	// }

	/* Read answer and write it to standard output */

	fd_set readfds;
	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 0;
	//Timeout with 0.0 timer is basically Polling

	for (;;) {
		memset(buf, '\0', sizeof(buf));
		//readfds set init
		FD_ZERO(&readfds);

		//add the conflicting fd's
		FD_SET(0, &readfds);
		FD_SET(sd, &readfds);

		//nfds = max_fd + 1
		select(sd+1, &readfds, NULL, NULL, &timeout);

		//If something is ready to be read from socket_fd
		if (FD_ISSET(sd, &readfds)){
			n = read(sd, buf, sizeof(buf));

			if (n < 0) {
				perror("Client read from peer");
				exit(1);
			}

			if (n <= 0)
				break;

			//decrypt data that was received from socket <-- server
			decrypt_data(crypto_fd);
			if (insist_write(1, buf, sizeof(buf)) != sizeof(buf)) {
				perror("Client wrote to stdout");
				exit(1);
			}
		}
		//If something is ready to be sent through socket --> server
		if (FD_ISSET(0, &readfds)){
			n = read(0, buf, sizeof(buf));

			if (n < 0) {
				perror("Client read from itself");
				exit(1);
			}

			if (n <= 0)
				break;

			//encrypt data that will be sent through socket --> server
			encrypt_data(crypto_fd);
			if (insist_write(sd, buf, sizeof(buf)) != sizeof(buf)) {
				perror("Client wrote to peer");
				exit(1);
			}
		}
	}
	fprintf(stderr, "\nDone.\n");

	if (ioctl(crypto_fd, CIOCFSESSION, &sess.ses)) { //Send IO command to end session
		perror("Error on ending Crypto Session");
		exit(1);
	}

	if (close(crypto_fd) < 0){ //close Cryptodev module
		perror("Erron on closing Cryptodev module FD");
		exit(1);
	}

	return 0;
}
