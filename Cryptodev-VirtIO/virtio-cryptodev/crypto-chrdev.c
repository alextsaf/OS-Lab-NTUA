/*
* crypto-chrdev.c
*
* Implementation of character devices
* for virtio-cryptodev device
*
* Vangelis Koukis <vkoukis@cslab.ece.ntua.gr>
* Dimitris Siakavaras <jimsiak@cslab.ece.ntua.gr>
* Stefanos Gerangelos <sgerag@cslab.ece.ntua.gr>
*
*/
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/virtio.h>
#include <linux/virtio_config.h>

#include "crypto.h"
#include "crypto-chrdev.h"
#include "debug.h"

#include "cryptodev.h"

/*
* Global data
*/
struct cdev crypto_chrdev_cdev;

/**
* Given the minor number of the inode return the crypto device
* that owns that number.
**/
static struct crypto_device *get_crypto_dev_by_minor(unsigned int minor)
{
	struct crypto_device *crdev;
	unsigned long flags;

	debug("Entering");

	spin_lock_irqsave(&crdrvdata.lock, flags);
	list_for_each_entry(crdev, &crdrvdata.devs, list) {
		if (crdev->minor == minor)
		goto out;
	}
	crdev = NULL;

	out:
	spin_unlock_irqrestore(&crdrvdata.lock, flags);

	debug("Leaving");
	return crdev;
}

/*************************************
* Implementation of file operations
* for the Crypto character device
*************************************/

static int crypto_chrdev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	int err;
	unsigned int len;
	struct crypto_open_file *crof;
	struct crypto_device *crdev;
	unsigned int *syscall_type;
	int *host_fd;

	debug("Entering");

	syscall_type = kzalloc(sizeof(*syscall_type), GFP_KERNEL);
	*syscall_type = VIRTIO_CRYPTODEV_SYSCALL_OPEN;
	host_fd = kzalloc(sizeof(*host_fd), GFP_KERNEL);
	*host_fd = -1;

	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
	goto fail;

	/* Associate this open file with the relevant crypto device. */
	crdev = get_crypto_dev_by_minor(iminor(inode));
	if (!crdev) {
		debug("Could not find crypto device with %u minor",
		iminor(inode));
		ret = -ENODEV;
		goto fail;
	}

	crof = kzalloc(sizeof(*crof), GFP_KERNEL);
	if (!crof) {
		ret = -ENOMEM;
		goto fail;
	}
	crof->crdev = crdev;
	crof->host_fd = -1;	//it changes on backend
	filp->private_data = crof;


	/**
	* We need two sg lists, one for syscall_type and one to get the
	* file descriptor from the host.
	**/
	struct scatterlist sgs_syscall_type, sgs_host_fd, *sgs[2];

	sg_init_one(&sgs_syscall_type, syscall_type, sizeof(syscall_type));
	sgs[0] = &sgs_syscall_type;
	sg_init_one(&sgs_host_fd, host_fd, sizeof(host_fd));
	sgs[1] = &sgs_host_fd;

	/**
	* Wait for the host to process our data.
	**/
	/* ?? */
	if (down_interruptible(&crdev->lock)){
		return -ERESTARTSYS;
	}

	virtqueue_add_sgs(crdev->vq, sgs, 1, 1, &sgs_syscall_type, GFP_ATOMIC); // TO ADD: TOKEN BUFFER
	debug("open: sent sgs to backend");
	virtqueue_kick(credv->vq);
	debug("open: notified host that new data has been sent (KICK)");

	while(!virtqueue_get_buf(crdev->vq, &len));

	up(&crdev->lock);

	crof->host_fd = *host_fd;

	//kaneis wait_event_interruptible() -> kaneis add sto queue -> kaneis kick


	/* If host failed to open() return -ENODEV. */

	if (crof->hostfd < 0){
		debug("Failed to open crypto device");
		ret = -ENODEV;
	}

	debug("Crypto device opened successfully");
	ret = 1;
	fail:
	debug("Leaving");
	return ret;
}

static int crypto_chrdev_release(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct crypto_open_file *crof = filp->private_data;
	struct crypto_device *crdev = crof->crdev;
	unsigned int *syscall_type;
	int *host_fd;

	debug("Entering crypto-release");

	syscall_type = kzalloc(sizeof(*syscall_type), GFP_KERNEL);
	*syscall_type = VIRTIO_CRYPTODEV_SYSCALL_CLOSE;

	host_fd = kzalloc(sizeof(host_fd), GFP_KERNEL)
	*host_fd = crof->host_fd;
	/**
	* Send data to the host.
	**/

	struct scatterlist sgs_syscall_type, sgs_host_fd, *sgs[2];

	sg_init_one(&sgs_syscall_type, syscall_type, sizeof(syscall_type));
	sgs[0] = &sgs_syscall_type;
	sg_init_one(&sgs_host_fd, host_fd, sizeof(host_fd));
	sgs[1] = &sgs_host_fd;

	/**
	* Wait for the host to process our data.
	**/

	if (down_interruptible(&crdev->lock)){
		return -ERESTARTSYS;
	}

	virtqueue_add_sgs(crdev->vq, sgs, 2, 0, &sgs_syscall_type, GFP_ATOMIC); // TO ADD: TOKEN BUFFER
	debug("open: sent sgs to backend");
	virtqueue_kick(credv->vq);
	debug("open: notified host that new data has been sent (KICK)");

	while(!virtqueue_get_buf(crdev->vq, &len));

	up(&crdev->lock);

	debug("Succesfully close cryptodev file");

	kfree(crof);
	debug("Leaving");
	return ret;

}

static long crypto_chrdev_ioctl(struct file *filp, unsigned int cmd,
	unsigned long arg)
	{
		long ret = 0;
		int err, *host_fd;

		struct crypto_open_file *crof = filp->private_data;
		struct crypto_device *crdev = crof->crdev;
		struct virtqueue *vq = crdev->vq;

		struct scatterlist sgs_syscall_type, sgs_host_fd,
		sgs_ioctl_cmd, sgs_key, sgs_sess_op, sgs_host_return_val,
		sgs_ses_id, sgs_crypt_op, sgs_src, sgs_iv, sgs_dst, *sgs[8];
		unsigned int num_out, num_in, len, *syscall_type, *ioctl_cmd;
		#define MSG_LEN 100
		unsigned char *output_msg, *input_msg;
		unsigned int *syscall_type, *ioctl_cmd;

		struct crypt_op *crypt;
		struct session_op *sess_op;
		unsigned char *sess_key;
		unsigned char *src, dst, iv;

		debug("Entering");

		/**
		* Allocate all data that will be sent to the host.
		**/
		syscall_type = kzalloc(sizeof(*syscall_type), GFP_KERNEL);
		*syscall_type = VIRTIO_CRYPTODEV_SYSCALL_IOCTL;

		host_fd = kzalloc(sizeof(*host_fd), GFP_KERNEL);
		*host_fd = crof->host_fd;

		num_out = 0;
		num_in = 0;

		/**
		*  These are common to all ioctl commands.
		**/
		sg_init_one(&sgs_syscall_type, syscall_type, sizeof(*syscall_type));
		sgs[num_out++] = &sgs_syscall_type;
		sg_init_one(&sgs_host_fd, &crof->host_fd, sizeof(host_fd));
		sgs[num_out++] = &sgs_host_fd;

		/**
		*  Add all the cmd specific sg lists.
		**/
		switch (cmd) {
			case CIOCGSESSION:
			debug("CIOCGSESSION");
			memcpy(output_msg, "Hello HOST from ioctl CIOCGSESSION.", 36);

			ioctl_cmd = kzalloc(sizeof(*ioctl_cmd), GFP_KERNEL);
			*ioctl_cmd = VIRTIO_CRYPTODEV_IOCTL_CIOCGSESSION;

			sg_init_one(&sgs_ioctl_cmd, ioctl_cmd, sizeof(ioctl_cmd));
			sgs[num_out++] = &sgs_ioctl_cmd;

			sess_key = kzalloc(sizeof(*sess_key), GFP_KERNEL);
			*sess_key = sess_op->key;
			//unsigned char session_key -> Read Flag
			sg_init_one(&sgs_key, sess_key, sizeof(key));
			sgs[num_out++] = &sgs_key;

			sess_op = kzalloc(sizeof(sess_op), GFP_KERNEL);
			//struct session_op session_op -> Write Flag
			sg_init_one(&sgs_sess_op, sess_op, sizeof(sess_op));
			sgs[num_in++ + num_out] = &sgs_sess_op;

			//int host_return_val -> Write Flag
			sg_init_one(&sgs_host_return_val, host_return_val, sizeof(host_return_val));
			sgs[num_in++ + num_out] = &sgs_host_return_val;

			break;

			case CIOCFSESSION:
			debug("CIOCFSESSION");
			memcpy(output_msg, "Hello HOST from ioctl CIOCFSESSION.", 36);

			*ioctl_cmd = VIRTIO_CRYPTODEV_IOCTL_CIOCFSESSION;
			sg_init_one(&sgs_ioctl_cmd, ioctl_cmd, sizeof(ioctl_cmd));
			sgs[num_out++] = &sgs_ioctl_cmd;

			ses_id = kzalloc(sizeof(uint32_t), GFP_KERNEL);
			//u32 ses id -> Read flag
			sg_init_one(&sgs_ses_id, ses_id, sizeof(ses_id));
			sgs[num_out++] = &sgs_ses_id;

			//int host_return_val -> Write Flag
			sg_init_one(&sgs_host_return_val, host_return_val, sizeof(host_return_val));
			sgs[num_in++ + num_out] = &sgs_host_return_val;

			break;

			case CIOCCRYPT:
			debug("CIOCCRYPT");
			memcpy(output_msg, "Hello HOST from ioctl CIOCCRYPT.", 33);

			*ioctl_cmd = VIRTIO_CRYPTODEV_IOCTL_CIOCCRYPT;
			sg_init_one(&sgs_ioctl_cmd, ioctl_cmd, sizeof(ioctl_cmd));
			sgs[num_out++] = &sgs_ioctl_cmd;

			crypt = kzalloc(sizeof(*crypt), GFP_KERNEL);
			sg_init_one(&sgs_crypt_op, crypt, sizeof(crypt));
			sgs[num_out++] = &sgs_crypt_op;

			src = kzalloc(crypt->len, GFP_KERNEL);
			sg_init_one(&sgs_src, src, sizeof(src));
			sgs[num_out++] = &sgs_src;

			iv = kzalloc(BLOCK_SIZE, GFP_KERNEL);
			sg_init_one(&sgs_iv, iv, sizeof(iv));
			sgs[num_out++] = &sgs_iv;

			dst = kzalloc(crypt->len, GFP_KERNEL);
			sg_init_one(&sgs_dst, dst, sizeof(dst));
			sgs[num_in++ + num_out] = &sgs_dst;

			//int host_return_val -> Write Flag
			sg_init_one(&sgs_host_return_val, host_return_val, sizeof(host_return_val));
			sgs[num_in++ + num_out] = &sgs_host_return_val;


			break;

			default:
			debug("Unsupported ioctl command");

			break;
		}


		/**
		* Wait for the host to process our data.
		**/
		/* ?? */
		/* ?? Lock ?? */
		err = virtqueue_add_sgs(vq, sgs, num_out, num_in,
			&sgs_syscall_type, GFP_ATOMIC);
			virtqueue_kick(vq);
			while (virtqueue_get_buf(vq, &len) == NULL)
			/* do nothing */;

			debug("We said: '%s'", output_msg);
			debug("Host answered: '%s'", input_msg);

			kfree(output_msg);
			kfree(input_msg);
			kfree(syscall_type);

			debug("Leaving");

			return ret;
		}

		static ssize_t crypto_chrdev_read(struct file *filp, char __user *usrbuf,
			size_t cnt, loff_t *f_pos)
			{
				debug("Entering");
				debug("Leaving");
				return -EINVAL;
			}

			static struct file_operations crypto_chrdev_fops =
			{
				.owner          = THIS_MODULE,
				.open           = crypto_chrdev_open,
				.release        = crypto_chrdev_release,
				.read           = crypto_chrdev_read,
				.unlocked_ioctl = crypto_chrdev_ioctl,
			};

			int crypto_chrdev_init(void)
			{
				int ret;
				dev_t dev_no;
				unsigned int crypto_minor_cnt = CRYPTO_NR_DEVICES;

				debug("Initializing character device...");
				cdev_init(&crypto_chrdev_cdev, &crypto_chrdev_fops);
				crypto_chrdev_cdev.owner = THIS_MODULE;

				dev_no = MKDEV(CRYPTO_CHRDEV_MAJOR, 0);
				ret = register_chrdev_region(dev_no, crypto_minor_cnt, "crypto_devs");
				if (ret < 0) {
					debug("failed to register region, ret = %d", ret);
					goto out;
				}
				ret = cdev_add(&crypto_chrdev_cdev, dev_no, crypto_minor_cnt);
				if (ret < 0) {
					debug("failed to add character device");
					goto out_with_chrdev_region;
				}

				debug("Completed successfully");
				return 0;

				out_with_chrdev_region:
				unregister_chrdev_region(dev_no, crypto_minor_cnt);
				out:
				return ret;
			}

			void crypto_chrdev_destroy(void)
			{
				dev_t dev_no;
				unsigned int crypto_minor_cnt = CRYPTO_NR_DEVICES;

				debug("entering");
				dev_no = MKDEV(CRYPTO_CHRDEV_MAJOR, 0);
				cdev_del(&crypto_chrdev_cdev);
				unregister_chrdev_region(dev_no, crypto_minor_cnt);
				debug("leaving");
			}
