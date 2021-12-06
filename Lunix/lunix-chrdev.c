/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * Alexandros Tsafos - el18211
 * Vasilis Vrettos - el18126
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;

	debug("state_needs_refresh got called");
	WARN_ON ( !(sensor = state->sensor));

	/* ? */
	return (sensor->msr_data[state->type]->last_update != state->buf_timestamp);

//state->type is an enum that is the type of the sensor. (lunix-module.c, lunix.h)
//last_update is the timestamp of the last time the sensor was updated through lunix_sensor_update (lunix-sensors.c)
//buf_timestamp (lunix-chrdev.h) is the last time the state buffer got "filled" with new data
//if the two timestamps are different, then, the state needs a refresh, so we return 1

	/* The following return is bogus, just for the stub to compile */
	//return 0; /* ? */
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;

	debug("leaving\n");
	WARN_ON ( !(sensor = state->sensor));


	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	/* ? */
	unsigned long state_flags;
	uint32_t temp_timestamp;
	uint16_t temp_values;
	int refresh;

	/* Why use spinlocks? See LDD3, p. 119 */
	debug("lunix_chrdev_state_update got called");
	spin_lock_irqsave(&sensor->lock, state_flags);

	//No spinlocks after reading :P
	//Code runs in intterrupt context, We need to disable intterrupts
	//We save the interrupt state. Better be safe than sorry :)


	/*
	 * Any new data available?
	 */
	/* ? */

	if (refresh = lunix_chrdev_state_needs_refresh(state)) {
		//if yes, store them, so no more race conditions occur (less spinlocks)
		temp_values = sensor->msr_data[state->type]->values[0];
		temp_timestamp = sensor->msr_data[state->type]->last_update;
	}

	spin_unlock_irqrestore(&sensor->lock, state_flags);
	debug("state needs refresh: %d\n", refresh);
	/*
	 * Now we can take our time to format them,
	 * holding only the private state semaphore
	 */



	/* ? */

	debug("leaving\n");
	return 0;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	/* ? */
	int ret, minor, sensor_type, sensor_nb;

	debug("entering\n");
	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;

   //nonseekable_open = open but for subsystems that do not want seekable file descriptors.
	 //inode represents file on disk

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */

	 minor = iminor(inode); //get minor number from C function --> inode gets us the /dev/sensor info

	 sensor_type = minor % 8; //
	 if (sensor_type >= N_LUNIX_MSR) goto out;

	 sensor_nb = minor / 8; //
	 debug("inode from /dev/sensor associated");

	 //buf_lim = int, buf_data = unsigned char (sizeof(20)), buf_timestamp = uint32_t
	 //lock = struct semaphore
	/* Allocate a new Lunix character device private state structure */
	/* ? */

	struct lunix_chrdev_state_struct *p_state;
	//allocate memory on kernel space for p_state struct --> GFP_KERNEL
	p_state = kzalloc(sizeof(struct lunix_chrdev_state_struct), GFP_KERNEL);
	if (!p_state) {
		printk(KERN_ERR "Failed to allocate memory for Lunix sensors\n");
		goto out;
	}
	//initialize p_state struct with values

	p_state->type = sensor_type;
	p_state->sensor = &(lunix_sensors[sensor_nb]);
	p_state->buf_timestamp = get_seconds(); //current timestamp
	p_state->buf_data[LUNIX_CHRDEV_BUFSZ - 1]='\0'; //initialised
	p_state->buf_lim = strnlen(p_state->buf_data, LUNIX_CHRDEV_BUFSZ);

	//initialize a semaphore
	sema_init(&p_state->lock,1);


	filp->private_data = p_state;
	//State struct must be private

	ret = 0; //everything is ok
	debug("State of type %d and sensor %d successfully associated\n", sensor_type, sensor_nb);

out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	kfree(filp->private_data);
	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	/* Why? */
	return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret;

	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;

	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);

	/* Lock? */
	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */
	if (*f_pos == 0) {
		while (lunix_chrdev_state_update(state) == -EAGAIN) {
			/* ? */
			/* The process needs to sleep */
			/* See LDD3, page 153 for a hint */
		}
	}

	/* End of file */
	/* ? */

	/* Determine the number of cached bytes to copy to userspace */
	/* ? */

	/* Auto-rewind on EOF mode? */
	/* ? */
out:
	/* Unlock? */
	return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops =
{
        .owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;

	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	/* ? */
	/* register_chrdev_region? */
	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}
	/* ? */
	/* cdev_add? */
	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
