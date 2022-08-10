#include <linux/fs.h> //文件操作相关structure的定义，file_operations
#include <linux/module.h> //模块加载卸载函数，module_init和module_exit
#include <linux/init.h> //初始化相关函数的定义
#include <linux/uaccess.h> //copy_to_user、copy_from_user 内核访问用户进程内存地址定义
#include <linux/cdev.h> //字符设备结构cdev相关函数定义
#include <linux/slab.h> //内存分配函数kcalloc、kzalloc的定义
#include <linux/device.h> //device、class结构的定义
#include <linux/fs.h> //文件操作相关structure的定义

#include <linux/platform_device.h> //结构体platform_device定义
#include <linux/mutex.h> //内核互斥锁
#include <linux/suspend.h> //内核休眠
#include <linux/pm_runtime.h> //电源管理

#define GLOBALMEM_SIZE 0x100 //global memory size，4KB,
#define DEVICE_NUM 99 //creat device node number 99，mknod /dev/char_driver c 99 0
#define DEVICE_NAME "char_driver" //device name
static int device_major = DEVICE_NUM;

struct char_driver_dev{
	struct cdev cdev; //cdev struct
	unsigned char mem[GLOBALMEM_SIZE]; //global memory
	struct mutex mutex; //mutex semaphore
};//device char_driver struct

struct char_driver_dev *char_driver_devp; //设备结构体指针

int char_driver_open(struct inode *inode,struct file *filp){
	
	filp->private_data = char_driver_devp; //set device struct pointer to file privatedata pointer
	
	return 0;
}

int char_driver_release(struct inode *inode,struct file *filp){
	
	return 0;
}

static ssize_t char_driver_read(struct file *filp,char __user *buf,size_t size){
	
	unsigned long p = *ppos;
	unsigned int count = size;
	int ret = 0;
	struct char_driver_dev *dev = filp->private_data; //get device struct pointer
	
	//analysis and classify the length
	if(p >= GLOBALMEM_SIZE)//overflow
		return 0;
	else if(count > GLOBALMEM_SIZE - p)
		count = GLOBALMEM_SIZE - p;
	
	//data flow derections ---- user buf -> kernel buf
	mutex_lock(&dev->mutex);
	if (copy_to_user(buf, (void *)(dev->mem + p), count))
        ret = -EFAULT;
    else {
        *ppos += count;
        ret = count;
        printk(KERN_INFO "read %d bytes from %ld\n", count, p);
    }
	mutex_unlock(&dev->mutex);
	return 0;
}

static ssize_t char_drive_write(struct file *filp, const char __user *buf,
                               size_t size, loff_t *ppos)
{
    unsigned long p = *ppos;
    unsigned int count = size;
    int ret = 0;
    struct char_drive_dev *dev = filp->private_data; // get device stuct pointer

    //analysis and classify the length
    if (p >= GLOBALMEM_SIZE) // write overflow
        return 0;

    if (count > GLOBALMEM_SIZE - p) // write count is too large
        count = GLOBALMEM_SIZE - p;

    //data flow derections ---- user buf -> kernel buf
	mutex_lock(&dev->mutex);
    if (copy_from_user(dev->mem + p, buf, count))
        ret = -EFAULT;
    else {
        *ppos += count;
        ret = count;
        printk(KERN_INFO "written %d bytes from %ld\n", count, p);
    }
	mutex_unlock(&dev->mutex);
    return ret;
}

static loff_t char_driver_llseek(struct file *filp,loff_t offset,int orig){
	
	loff_t ret = 0;
	
	switch (orig){
		
		case 0: // from the file head
			if(offset < 0 || ((unsigned int)offset > GLOBALMEM_SIZE)){
				
				ret = -EINVAL;
				break;	
			}
			else{
				filp->f_pos = (unsigned int)offset;
				ret = filp->f_pos;
				break;
			}
		case 1:// from current position
			if(offset < 0 || ((unsigned int)offset > GLOBALMEM_SIZE)){
				
				ret = -EINVAL;
				break;	
			}
			else{
				filp->f_pos += (unsigned int)offset;
				ret = filp->f_pos;
				break;
			}
	}
	
	return ret;
	
}

static long char_dev_ioctl(struct file *filp,unsign int cmd,unsigned long arg){
	
	struct char_driver_dev *dev = filp->private_data; //get pointer
	
	switch (cmd){
		case MEM_CLEAR:
			memset(dev->mem,0,GLOBALMEM_SIZE);
			printk(KERN_INFO "globalmem is set to zero\n");
			break;
		default:
			return -EINVAL;
	}
	
	return 0;
}

//file operations struct

static const struct file_operations char_driver_fops = {
    .owner = THIS_MODULE,
    .llseek = char_driver_llseek,
    .read = char_driver_read,
    .write = char_driver_write,
    .open = char_driver_open,
    .release = char_driver_release,
    .unlocked_ioctl = char_driver_ioctl,
};

static void char_driver_setup_cdev(struct char_driver_dev *dev,int index){
	
	int err;
	int devno = MKDEV(device_major,0);
	
	cdev_init(&dev->cdev,&char_driver_fops);
	err = cdev_add(&dev->cdev,devon,1);
	if(err)
		printk(KERN_NOTICE "Error %d adding globalmem", err);
	
}


//加载函数
static int __init char_driver_init(void){
	
	int ret;
	dev_t devno = MKDEV(device_major,0);
	
	if(device_major){
		ret= register_chrdev(devno,1,DEVICE_NAME);//字符设备注册
	}
	else{
		 // get major no dynamically
        ret = alloc_chrdev_region(&devno, 0, 1, DEVICE_NAME);
        char_drive_major = MAJOR(devno);
	}

	if(ret < 0){
		printk("char_driver Regist Fail!\n");
	}
	else{
		printk("char_driver Regist Success!\n");
		device_major = ret;
		printk("Major=%d\n",device_major);
	}
	
	char_driver_devp = kmalloc(sizeof(struct char_drive_dev), GFP_KERNEL);
    if (!char_drive_devp) {
        result = - ENOMEM;
        goto fail_malloc;
    }

    memset(char_drive_devp, 0, sizeof(struct char_drive_dev));
    char_drive_setup_cdev(char_drive_devp, 0);

    return 0;

	fail_malloc:
		unregister_chrdev_region(devno, 1);
		return result;
	
	
	return ret;
}

//卸载函数

static void __exit char_driver_exit(void){
	
	dev_t devid;
	int i;
	for(i=0;i<DEVICE_NUM;i++){
		cdev_del(&(char_driver_devp+i)->cdev);
		devid = MKDEV(char_major,i);
		device_destory(,devid);
	}
	kfree(char_driver_devp);
	
	unregister_chrdev_region(MKDEV(char_major,0),DEVICE_NUM);
	
}

module_init(char_driver_init);
module_exit(char_driver_exit);
MODULE_AUTHOR("xianglong.zheng");
MODULE_DESCRIPTION("char_dev_driver");
MODULE_VERSION("FIRST");
