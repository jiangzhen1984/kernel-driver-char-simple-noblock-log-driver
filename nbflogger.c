
#include "nbflogger.h"

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/cdev.h>


#define MAX_LOGGER_BUF_SIZE (1000)

#define DEVICE_COUNT (1)
#define DEVICE_NAME ("nbflogger")

struct nbf_logger {
    struct device * p_d;
    struct class * p_c;
    dev_t d_t;
    struct cdev c_d;
    char buf[MAX_LOGGER_BUF_SIZE]; 
    unsigned int head_pos;
    unsigned int offset;
    struct mutex w_lock;
    struct list_head owners;
} lg;


struct logger_owner {
    struct list_head list; 
    unsigned int offset; 
};

struct nbf_logger * g_logger = & lg;



static int nbf_logger_open(struct inode * inode, struct file * file)
{
    printk(KERN_INFO" new owner open device\n");
    struct logger_owner * lo = kmalloc(sizeof(struct logger_owner), GFP_KERNEL);
    if (!lo) {
        return -ENOMEM;
    }

    lo->offset = 0;
    INIT_LIST_HEAD(&lo->list);
    mutex_lock(&g_logger->w_lock);
    list_add_tail(&lo->list, &g_logger->owners);
    mutex_unlock(&g_logger->w_lock);

    file->private_data = lo;
    
    return 0;
}


static int nbf_logger_release(struct inode * inode, struct file * file)
{
    struct logger_owner * lo = (struct logger_owner *) file->private_data;
    mutex_lock(&g_logger->w_lock);
    list_del(&lo->list);
    mutex_unlock(&g_logger->w_lock);
    kfree(lo);
    return 0;
}


static ssize_t nbf_logger_read(struct file * file, char __user *buf,
                    size_t count, loff_t * pos)
{
    return 0;
}

static ssize_t nbf_logger_write(struct file * file, const char __user *buf,
                    size_t count, loff_t * pos)
{
    return 0;
}


static long nbf_logger_ioctl(struct file * file,
                    unsigned int cmd, unsigned long arg)
{

    printk(KERN_INFO" get ioctl operation %d\n", cmd);
    printk("------------------------------------------\n");
    switch (cmd) {
        case NBF_LOGGER_IOCTL_SET_IO:
            printk(KERN_INFO" get ioctl set io command %d\n", cmd);
            break;
        case NBF_LOGGER_IOCTL_GET_IO:
            printk(KERN_INFO" get ioctl get io command %d\n", cmd);
            break;
        default:
            printk(KERN_ERR" unkonwn command %d\n", cmd);
    }

    return 0;
}

static const struct file_operations ops = {
    .owner      = THIS_MODULE,
    .open       = nbf_logger_open,
    .release    = nbf_logger_release,
    .read       = nbf_logger_read,
    .write      = nbf_logger_write,
    .unlocked_ioctl       = nbf_logger_ioctl,
    .compat_ioctl       = nbf_logger_ioctl,
};

static int __init nbf_logger_init(void)
{
    int ret = 0;
    
    ret = alloc_chrdev_region(&g_logger->d_t, 0, DEVICE_COUNT, DEVICE_NAME);
    if (ret) {
        printk(KERN_ERR" can't alloct dev_t %d\n", ret);
        return ret;
    }

    g_logger->p_c = class_create(THIS_MODULE, DEVICE_NAME);
    if (IS_ERR(g_logger->p_c)) {
        printk(KERN_ERR" can't create class for %s\n", DEVICE_NAME); 
        ret = -ENOMEM;
        goto destroy_chrdev;
    }

    g_logger->p_d = device_create(g_logger->p_c, NULL, g_logger->d_t, NULL, DEVICE_NAME);
    if (IS_ERR(g_logger->p_d)) {
        ret = -ENOMEM;
        printk(KERN_ERR" can't create device file err(%s) \n", DEVICE_NAME);
        goto destroy_class;
    }
    

    cdev_init(&g_logger->c_d, &ops);

    ret = cdev_add(&g_logger->c_d, g_logger->d_t, DEVICE_COUNT);
    if (ret) {
        printk(KERN_ERR" can't add device to tree err(%s) \n", DEVICE_NAME);
        goto destroy_device;
    }

    mutex_init(&g_logger->w_lock);
    g_logger->head_pos = 0;
    g_logger->offset = 0;
    INIT_LIST_HEAD(&g_logger->owners);

    return ret;
destroy_device:
    if (g_logger->p_d) {
        device_destroy(g_logger->p_c, g_logger->d_t);
    }
destroy_class:
    if (g_logger->p_c) {
        class_destroy(g_logger->p_c);
    }
destroy_chrdev:
    unregister_chrdev_region(g_logger->d_t, DEVICE_COUNT);
    return ret;
}

static void __exit nbf_logger_exit(void)
{
    mutex_destroy(&g_logger->w_lock);
    cdev_del(&g_logger->c_d);

    if (g_logger->p_d) {
        device_destroy(g_logger->p_c, g_logger->d_t);
    }

    if (g_logger->p_c) {
        class_destroy(g_logger->p_c);
    }

    unregister_chrdev_region(g_logger->d_t, DEVICE_COUNT);
}


module_init(nbf_logger_init);
module_exit(nbf_logger_exit);
MODULE_LICENSE ("GPL v2");
