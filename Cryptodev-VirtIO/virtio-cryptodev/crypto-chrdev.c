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

#define MSG_LEN 100

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
	struct scatterlist sgs_syscall_type, sgs_host_fd, *sgs[2];

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
	virtqueue_kick(crdev->vq);
	debug("open: notified host that new data has been sent (KICK)");

	while(!virtqueue_get_buf(crdev->vq, &len));

	up(&crdev->lock);

	crof->host_fd = *host_fd;

	//kaneis wait_event_interruptible() -> kaneis add sto queue -> kaneis kick


	/* If host failed to open() return -ENODEV. */

	if (crof->host_fd < 0){
		debug("Failed to open crypto device");
		ret = -ENODEV;
	}

	debug("Crypto device opened successfully");

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
	//int *host_fd;
	unsigned int len;
	struct scatterlist sgs_syscall_type, sgs_host_fd, *sgs[2];

	debug("Entering crypto-release");

	syscall_type = kzalloc(sizeof(*syscall_type), GFP_KERNEL);
	*syscall_type = VIRTIO_CRYPTODEV_SYSCALL_CLOSE;

	//host_fd = kzalloc(sizeof(host_fd), GFP_KERNEL)
	//*host_fd = crof->host_fd;
	/**
	* Send data to the host.
	**/



	sg_init_one(&sgs_syscall_type, syscall_type, sizeof(syscall_type));
	sgs[0] = &sgs_syscall_type;
	sg_init_one(&sgs_host_fd, &crof->host_fd, sizeof(crof->host_fd));
	sgs[1] = &sgs_host_fd;

	/**
	* Wait for the host to process our data.
	**/

	if (down_interruptible(&crdev->lock)){
		return -ERESTARTSYS;
	}

	virtqueue_add_sgs(crdev->vq, sgs, 2, 0, &sgs_syscall_type, GFP_ATOMIC); // TO ADD: TOKEN BUFFER
	debug("open: sent sgs to backend");
	virtqueue_kick(crdev->vq);
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
		int err;
		long *host_return_val;

		struct crypto_open_file *crof = filp->private_data;
		struct crypto_device *crdev = crof->crdev;
		struct virtqueue *vq = crdev->vq;

		struct scatterlist sgs_syscall_type, sgs_host_fd,
		sgs_ioctl_cmd, sgs_key, sgs_sess_op, sgs_host_return_val,
		sgs_sess_id, sgs_crypt_op, sgs_src, sgs_iv, sgs_dst, *sgs[8];
		unsigned int num_out, num_in, len, *syscall_type, *ioctl_cmd;


		//Need to be initialized to NULL or else kfree() complains.
		uint32_t *sess_id = NULL;
		struct crypt_op *crypt = NULL;
		struct session_op *sess_op = NULL;
		unsigned char *sess_key = NULL;
		unsigned char *src = NULL, *dst = NULL, *iv = NULL;

		debug("Entering frontend ioctl");

		/**
		* Allocate all data that will be sent to the host.
		**/
		syscall_type = kzalloc(sizeof(*syscall_type), GFP_KERNEL);
		if (!syscall_type){
			ret = -ENOMEM;
			printk(KERN_ERR "Failed to allocate memory for syscall_type\n");
		}
		*syscall_type = VIRTIO_CRYPTODEV_SYSCALL_IOCTL;

		 host_fd = kzalloc(sizeof(*host_fd), GFP_KERNEL);
		 if (!host_fd){
		 	ret = -ENOMEM;
		 	printk(KERN_ERR "Failed to allocate memory for host_fd\n");
		 }
		*host_fd = crof->host_fd;

		ioctl_cmd = kzalloc(sizeof(*ioctl_cmd), GFP_KERNEL);
		if (!ioctl_cmd){
			ret = -ENOMEM;
			printk(KERN_ERR "Failed to allocate memory for ioctl_cmd\n");
		}
		//write sgs
		host_return_val = kzalloc(sizeof(*host_return_val), GFP_KERNEL);
		if (!host_return_val){
			ret = -ENOMEM;
			printk(KERN_ERR "Failed to allocate memory for host_return_val\n");
		}

		num_out = 0;
		num_in = 0;

		/**
		*  These are common to all ioctl commands.
		**/
		sg_init_one(&sgs_syscall_type, syscall_type, sizeof(*syscall_type));
		sgs[num_out++] = &sgs_syscall_type;
		sg_init_one(&sgs_host_fd, host_fd, sizeof(host_fd));
		sgs[num_out++] = &sgs_host_fd;

		/**
		*  Add all the cmd specific sg lists.
		**/
		switch (cmd) {
			case CIOCGSESSION:
			debug("frontend CIOCGSESSION");
			*ioctl_cmd = VIRTIO_CRYPTODEV_IOCTL_CIOCGSESSION;
			sg_init_one(&sgs_ioctl_cmd, ioctl_cmd, sizeof(ioctl_cmd));
			sgs[num_out++] = &sgs_ioctl_cmd;
			debug("ioctl scatterlist init successful");

			//arg will contain the session we want to open
			sess_op = kzalloc(sizeof(sess_op), GFP_KERNEL);
			if (!sess_op){
				ret = -ENOMEM;
				printk(KERN_ERR "Failed to allocate memory for session_op\n");
				goto fail;
			}

			if (copy_from_user(sess_op, (struct session_op*)arg, sizeof(*sess_op)))
			{
				debug("CIOCGSESSION: copy_from_user failed (session)");
				ret = -EFAULT;
				goto fail;
			}

			sess_key = kzalloc(sess_op->keylen*sizeof(char), GFP_KERNEL);
			if (!sess_key){
				ret = -ENOMEM;
				printk(KERN_ERR "Failed to allocate memory for sess_key\n");
			}

			if (copy_from_user(sess_key, sess_op->key, sess_op->keylen*sizeof(char)))
			{
				debug("CIOCGSESSION: copy_from_user failed (session key)");
				ret = -EFAULT;
				goto fail;
			}
			//unsigned char session_key -> Read Flag

			sg_init_one(&sgs_key, sess_key, sizeof(sess_key));
			sgs[num_out++] = &sgs_key;
			debug("sess key scatterlist init successful");
			//struct session_op session_op -> Write Flag
			sg_init_one(&sgs_sess_op, sess_op, sizeof(sess_op));
			sgs[num_in++ + num_out] = &sgs_sess_op;
			debug("sess op scatterlist init successful");
			//int host_return_val -> Write Flag
			sg_init_one(&sgs_host_return_val, host_return_val, sizeof(host_return_val));
			sgs[num_in++ + num_out] = &sgs_host_return_val;
			debug("host_ret_val scatterlist init successful");
			break;

			case CIOCFSESSION:
			debug("CIOCFSESSION");


			*ioctl_cmd = VIRTIO_CRYPTODEV_IOCTL_CIOCFSESSION;
			sg_init_one(&sgs_ioctl_cmd, ioctl_cmd, sizeof(ioctl_cmd));
			sgs[num_out++] = &sgs_ioctl_cmd;

			sess_id = kzalloc(sizeof(uint32_t), GFP_KERNEL);
			if (!sess_id){
				ret = -ENOMEM;
				printk(KERN_ERR "Failed to allocate memory for sess_id\n");
			}
			if (copy_from_user(sess_id, (uint32_t *)arg, sizeof(sess_id)))
			{
				debug("CIOCFSESSION: copy_from_user failed (session ID)");
				ret = -EFAULT;
				goto fail;
			}
			//u32 ses id -> Read flag
			sg_init_one(&sgs_sess_id, sess_id, sizeof(sess_id));
			sgs[num_out++] = &sgs_sess_id;

			//int host_return_val -> Write Flag
			sg_init_one(&sgs_host_return_val, host_return_val, sizeof(host_return_val));
			sgs[num_in++ + num_out] = &sgs_host_return_val;

			break;

			case CIOCCRYPT:
			debug("CIOCCRYPT");


			*ioctl_cmd = VIRTIO_CRYPTODEV_IOCTL_CIOCCRYPT;
			sg_init_one(&sgs_ioctl_cmd, ioctl_cmd, sizeof(ioctl_cmd));
			sgs[num_out++] = &sgs_ioctl_cmd;

			crypt = kzalloc(sizeof(*crypt), GFP_KERNEL);
			if (!crypt){
				ret = -ENOMEM;
				printk(KERN_ERR "Failed to allocate memory for crypt\n");
			}
			if (copy_from_user(crypt, (struct crypt_op *)arg, sizeof(*crypt)))
			{
				debug("CIOCCRYPT: copy_from_user failed (crypto)");
				ret = -EFAULT;
				goto fail;
			}
			sg_init_one(&sgs_crypt_op, crypt, sizeof(crypt));
			sgs[num_out++] = &sgs_crypt_op;

			src = kzalloc(sizeof(char)*crypt->len, GFP_KERNEL);
			if (!src){
				ret = -ENOMEM;
				printk(KERN_ERR "Failed to allocate memory for src\n");
			}
			if (copy_from_user(src, crypt->src, sizeof(char)*crypt->len))
			{
				debug("CIOCCRYPT: copy_from_user failed (source)");
				ret = -EFAULT;
				goto fail;
			}
			sg_init_one(&sgs_src, src, sizeof(src));
			sgs[num_out++] = &sgs_src;

			iv = kzalloc(BLOCK_SIZE * sizeof(char), GFP_KERNEL);
			if (!iv){
				ret = -ENOMEM;
				printk(KERN_ERR "Failed to allocate memory for iv\n");
			}

			if (copy_from_user(iv, crypt->iv, BLOCK_SIZE * sizeof(char)))
			{
				debug("CIOCCRYPT: copy_from_user failed (iv)");
				ret = -EFAULT;
				goto fail;
			}
			sg_init_one(&sgs_iv, iv, sizeof(iv));
			sgs[num_out++] = &sgs_iv;

			dst = kzalloc(crypt->len * sizeof(char), GFP_KERNEL);
			if (!dst){
				ret = -ENOMEM;
				printk(KERN_ERR "Failed to allocate memory for dst\n");
			}
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
		if (down_interruptible(&crdev->lock)){
			return -ERESTARTSYS;
		}
		debug("locked before sending to backend (ioctl)");
		err = virtqueue_add_sgs(vq, sgs, num_out, num_in,
			&sgs_syscall_type, GFP_ATOMIC);
		debug("sent queue to backend (ioctl)");
		virtqueue_kick(vq);
		debug("informed backend for change in VQ (kick_ioctl)");
		while (virtqueue_get_buf(vq, &len) == NULL)
			/* do nothing */;

		up(&crdev->lock);
		debug("unlocked (ioctl)");
		switch (cmd) {
			case CIOCGSESSION:
			debug("before copy_to_user in CIOCGSESSION ioctl");
				if (copy_to_user((struct session_op*) arg, sess_op, sizeof(*sess_op))){
					debug("CIOCGSESSION: copy_to_user failed (session)");
					ret = -EFAULT;
					goto fail;
				}
				break;

			case CIOCFSESSION:
				if (copy_to_user((uint32_t *) arg, sess_id, sizeof(*sess_id))) {
					debug("CIOCFSESSION: copy_to_user failed (sess_id)");
					ret = -EFAULT;
					goto fail;
				}
				break;

			case CIOCCRYPT:
				if (copy_to_user(((struct crypt_op *)arg)->dst, dst, crypt->len)) {
					debug("CIOCCRYPT: copy_to_user (dst)");
					ret = -EFAULT;
					goto fail;
				}
				break;

			default: break;
		}
	fail:
			debug("about to free memory");
			kfree(crypt);
			kfree(dst);
			kfree(iv);
			kfree(src);
			kfree(host_return_val);
			kfree(syscall_type);
			kfree(ioctl_cmd);
			kfree(sess_key);
			kfree(sess_id);
			kfree(sess_op);


			//DO NOT kfree(host_fd);!!!!!

			debug("Leaving frontend ioctl");


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
