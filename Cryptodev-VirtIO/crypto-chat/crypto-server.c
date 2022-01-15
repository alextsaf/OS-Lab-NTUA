/*
* socket-server.c
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

#include <crypto/cryptodev.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include "crypto-common.h"

#define DATA_SIZE       256
#define BLOCK_SIZE      16
#define KEY_SIZE	16  /* AES128 */

//initializations
unsigned char buf[DATA_SIZE];
struct session_op sess;
unsigned char key[KEY_SIZE] = "mpougatsiarides1";
unsigned char iv[BLOCK_SIZE] = "hsfairaponanana1";

/* Convert a buffer to upercase */
void toupper_buf(char *buf, size_t n)
{
	size_t i;

	for (i = 0; i < n; i++)
	buf[i] = toupper(buf[i]);
}

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

for (i = 0; i < sizeof(buf); i++) {
	buf[i] = data.decrypted[i];
}
//Function returns 0 on success
return 0;
}

int main(void)
{

	char addrstr[INET_ADDRSTRLEN];
	int sd, newsd;
	ssize_t n;
	socklen_t len;
	struct sockaddr_in sa;


	memset(&sess, 0, sizeof(sess));
	/* Make sure a broken connection doesn't kill us */
	signal(SIGPIPE, SIG_IGN);

	/* Create TCP/IP socket, used as main chat channel */
	if ((sd = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
		perror("socket");
		exit(1);
	}
	fprintf(stderr, "Created TCP socket\n");

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

	if (ioctl(crypto_fd, CIOCGSESSION, &sess)) {
		perror("Crypto session init failed");
		return 1;
	}

	/* Bind to a well-known port */
	memset(&sa, 0, sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_port = htons(TCP_PORT);
	sa.sin_addr.s_addr = htonl(INADDR_ANY);
	if (bind(sd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
		perror("bind");
		//maybe unbind?
		exit(1);
	}
	fprintf(stderr, "Bound TCP socket to port %d\n", TCP_PORT);

	/* Listen for incoming connections */
	if (listen(sd, TCP_BACKLOG) < 0) {
		perror("listen");
		exit(1);
	}

	/* Loop forever, accept()ing connections */
	for (;;) {
		fprintf(stderr, "Waiting for an incoming connection...\n");

		/* Accept an incoming connection */
		len = sizeof(struct sockaddr_in);
		if ((newsd = accept(sd, (struct sockaddr *)&sa, &len)) < 0) {
			perror("accept");
			exit(1);
		}
		//convert IPv4 and IPv6 addresses from binary to text form
		if (!inet_ntop(AF_INET, &sa.sin_addr, addrstr, sizeof(addrstr))) {
			perror("could not format IP address");
			exit(1);
		}
		fprintf(stderr, "Incoming connection from %s:%d\n",
		addrstr, ntohs(sa.sin_port));

		fd_set readfds;
		struct timeval timeout;
		timeout.tv_sec = 0;
		timeout.tv_usec = 0;

		/* We break out of the loop when the remote peer goes away */
		for (;;) {
			memset(buf, '\0', sizeof(buf));

			//readfds set init
			FD_ZERO(&readfds);

			//add the conflicting fd's
			FD_SET(0, &readfds);
			FD_SET(newsd, &readfds);

			//see which fd has something to say
			select(newsd+1, &readfds, NULL, NULL, &timeout);

			if (FD_ISSET(newsd, &readfds)){
				n = read(newsd, buf, sizeof(buf));
				if (n <= 0) {
					if (n < 0)
					perror("read from remote peer failed");
					else
					fprintf(stderr, "Peer went away\n");
					break;
				}
				//	toupper_buf(buf, n);
				decrypt_data(crypto_fd);
				if (insist_write(1, buf, sizeof(buf)) != sizeof(buf)) {
					perror("write to remote peer failed");
					break;
				}
			}
			if (FD_ISSET(0, &readfds)){

				n = read(0, buf, sizeof(buf));
				if (n <= 0) {
					if (n < 0)
					perror("read from remote peer failed");
					else
					fprintf(stderr, "Peer went away\n");
					break;
				}

				encrypt_data(crypto_fd);

				if (insist_write(newsd, buf, sizeof(buf)) != sizeof(buf)) {
					perror("write to remote peer failed");
					break;
				}
			}
		}
		/* Make sure we don't leak open files */
		if (close(newsd) < 0)
		perror("close");
	}

	if (ioctl(crypto_fd, CIOCFSESSION, &sess.ses)) {
		perror("Error on ending Crypto Session");
		exit(1);
	}
	if (close(crypto_fd) < 0){
		perror("Erron on closing Cryptodev module FD");
		exit(1);
	}
	/* This will never happen */
	return 1;
}
