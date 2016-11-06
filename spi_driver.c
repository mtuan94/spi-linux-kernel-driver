
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/random.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/spinlock.h>

// example SPI device registers
#define SOMEKINDOFRESETCOMMAND 0
#define SOMEREG 0
#define SPIDEVDATAREG 0

//// khai báo interupt cho gpio19
// our interrupt name for the GPIO19 pin interrupt
#define GPIO19_INT_NAME  "gpio19_int"

//// tính ra device pin từ gpio pin
// this is used to calculate device pin value from the GPIO pin value
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

//// sử dụng 2 gpio là 19 và 21
// our two used GPIO pins
#define BEAGLEBONE_GPIO19 GPIO_TO_PIN(3, 19)
#define BEAGLEBONE_GPIO21 GPIO_TO_PIN(3, 21)

// GPIO pins of the two used user LEDs - LED 3 AND LED4
#define BEAGLEBONE_USR3_LED GPIO_TO_PIN(1, 23)
#define BEAGLEBONE_USR4_LED GPIO_TO_PIN(1, 24)

// macros for turning the LEDs on and off
#define BEAGLEBONE_LED3ON gpio_set_value(BEAGLEBONE_USR3_LED, 255)
#define BEAGLEBONE_LED3OFF gpio_set_value(BEAGLEBONE_USR3_LED, 0)
#define BEAGLEBONE_LED4ON gpio_set_value(BEAGLEBONE_USR4_LED, 255)
#define BEAGLEBONE_LED4OFF gpio_set_value(BEAGLEBONE_USR4_LED, 0)

// macros to check the values of the GPIO pins
#define BEAGLEBONE_GPIO19_HIGH gpio_get_value(BEAGLEBONE_GPIO19)
#define BEAGLEBONE_GPIO19_LOW (gpio_get_value(BEAGLEBONE_GPIO19) == 0)
#define BEAGLEBONE_GPIO21_HIGH gpio_get_value(BEAGLEBONE_GPIO21)
#define BEAGLEBONE_GPIO21_LOW (gpio_get_value(BEAGLEBONE_GPIO21) == 0)

static struct kobject *spikernmodex_kobj; // this is used for the sysfs entries
static struct spi_device *myspi;

//// FIFO LƯU TRỮ DATA TỪ SYSFS FILE
// this FIFO store is used for storing incoming data from the sysfs file
// this stored data gets sent to the SPI device
#define FIFOSTORESIZE 20
#define FIFOSTOREDATASIZE 64
typedef struct {
	uint8_t data[FIFOSTOREDATASIZE];
	int size;
} fifostoreentry;
static fifostoreentry fifostore[FIFOSTORESIZE];
static int fifostorepos = 0;


//// BUFER LƯU TRỮ DATA TỪ SPI DEVICE
// this ringbuffer is used for storing incoming data from the SPI device
// when the user reads the sysfs file, this ringbuffer's contents get printed out
#define RINGBUFFERSIZE 20
#define RINGBUFFERDATASIZE 64
typedef struct {
	int used;
	int completed;
	uint8_t data[RINGBUFFERDATASIZE];
	int size;
} ringbufferentry;
static ringbufferentry ringbuffer[RINGBUFFERSIZE];
static int ringbufferpos = 0;

static struct workqueue_struct* spikernmodex_workqueue;
static struct work_struct spikernmodex_work_processfifostore;
static struct work_struct spikernmodex_work_read;

//// FUNCTION DISABLE INTERUPT KHI SPI_SYNC ĐANG HOẠT ĐỘNG
// this spinlock is used for disabling interrupts while spi_sync() is running
static DEFINE_SPINLOCK(spilock);
static unsigned long spilockflags;


//// SEND DATA TỪ THANH GHI REG VÀ NHẬN DATA TỪ VAL
// this function sends the register value "reg", then "val", then returns with a read byte
// all SPI calls are implemented using spi_sync because the callback function given to
// spi_async() didn't got called for me on the BeagleBone
static uint8_t spi_write_reg(uint8_t reg, uint8_t val)
{
	struct spi_transfer t[3];
	struct spi_message m;
	uint8_t rxbuf;

	spi_message_init(&m);

	memset(t, 0, sizeof(t));

	t[0].tx_buf = &reg;
	t[0].len = 1;
	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = &val;
	t[1].len = 1;
	spi_message_add_tail(&t[1], &m);

	t[2].rx_buf = &rxbuf;
	t[2].len = 1;
	spi_message_add_tail(&t[2], &m);

	spin_lock_irqsave(&spilock, spilockflags);
	spi_sync(myspi, &m);
	spin_unlock_irqrestore(&spilock, spilockflags);

	return rxbuf;
}

//// GHI (COUNT) BYTES TỪ (BUFF) VÀO THANH GHI NÀO ĐÓ
// this function writes "count" bytes from "buf" to the given "reg" register
static void spi_write_reg_burst(uint8_t reg, const uint8_t *buf, size_t count)
{
	struct spi_transfer t[2];
	struct spi_message m;

	spi_message_init(&m);

	memset(t, 0, sizeof(t));

	t[0].tx_buf = &reg;
	t[0].len = 1;
	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = buf;
	t[1].len = count;
	spi_message_add_tail(&t[1], &m);

	spin_lock_irqsave(&spilock, spilockflags);
	spi_sync(myspi, &m);
	spin_unlock_irqrestore(&spilock, spilockflags);
}

//// ĐỌC (COUNT) BYTES TỪ THANH GHI NÀO ĐÓ VÀO VỊ TRÍ NÀO ĐÓ
// this function reads "count" bytes from the "reg" register to "dst"
static uint8_t spi_read_reg_burst(uint8_t reg, uint8_t *dst, size_t count)
{
	struct spi_transfer t[2];
	struct spi_message m;

	spi_message_init(&m);

	memset(t, 0, sizeof(t));

	t[0].tx_buf = &reg;
	t[0].len = 1;
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = (uint8_t *)dst;
	t[1].len = count;
	spi_message_add_tail(&t[1], &m);

	spin_lock_irqsave(&spilock, spilockflags);
	spi_sync(myspi, &m);
	spin_unlock_irqrestore(&spilock, spilockflags);

	return 0;
}

//// TRẢ VỀ GIÁ TRỊ CỦA THANH GHI
// this wrapper function returns the value of register "reg"
static uint8_t spi_read_reg(uint8_t reg)
{
	uint8_t res;

	if (spi_read_reg_burst(reg, &res, 1) == 0)
		return res;

	return 0;
}

/// THỰC HIỆN COMMAND VÀ TRẢ VỀ GIÁ TRỊ MÀ SPI DEVICE TRẢ VỀ
// this wrapper function executes the cmd command and returns with the value returned by the SPI device
static uint8_t spi_cmd(uint8_t cmd)
{
	return spi_read_reg(cmd);
}

//// GHI RA "STORE" CỦA SYSFS FILE
// this function is called when writing to the "store" sysfs file
static ssize_t spikernmodex_store_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	if (count > FIFOSTOREDATASIZE) {
		printk(KERN_ERR "can't store data because it's too big.");
		return 0;
	}

	if (fifostorepos >= FIFOSTORESIZE) {
		printk(KERN_ERR "can't store data because fifo is full.");
		return 0;
	}

	printk(KERN_DEBUG "store_store(): storing %d bytes to store pos 0x%.2x\n", count, fifostorepos);

	memcpy(fifostore[fifostorepos].data, buf, count);
	fifostore[fifostorepos].size = count;
	fifostorepos++;

	printk(KERN_DEBUG "queueing work PROCESSFIFOSTORE\n");
	queue_work(spikernmodex_workqueue, &spikernmodex_work_processfifostore);

	return count;
}

//// ĐỌC VÀO TỪ "STORE" CỦA SYSFS FILE
// this function is called when reading from the "store" sysfs file
static ssize_t spikernmodex_store_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int currentbufsize;
	int i = ringbufferpos+1;

	if (i == RINGBUFFERSIZE)
		i = 0;

	while (i != ringbufferpos) {
		if (ringbuffer[i].completed) {
			currentbufsize = ringbuffer[i].size;
			// we found a used & completed slot, outputting
			printk(KERN_DEBUG "store_show(): outputting ringbuf %.2x, %d bytes\n", i, currentbufsize);
			memcpy(buf, ringbuffer[i].data, currentbufsize);
			ringbuffer[i].completed = ringbuffer[i].used = 0;
			return currentbufsize;
		}

		i++;
		if (i == RINGBUFFERSIZE)
			i = 0;
	}

    return 0;
}

//// WORK QUEQUE INTERVAL
// LẤY TOÀN BỘ DATA STORED TRONG FIFO ĐẨY VÀO SPI 						[ FIFO -> SPI DEVICE ]
// LẤY TOÀN BỘ DATA TRONG SPI DEVICE ĐẨY VÀO RINGBUFFER				[ SPI DEVICE -> RINGBUFFER ]
// - flow là thế này, khi có data từ thiết bị khác gửi đến qua spi, nó sẽ dc đẩy vào ringbuffer để xử lý
// - quá trình xử lý bla bla ../ data cần gửi đi qua spi sẽ đc tống vào fifo để chuẩn bị đẩy đi
void spikernmodex_workqueue_handler(struct work_struct *work) {
	int i, j;

	// this work outputs all data stored in the FIFO to the SPI device
	if (work == &spikernmodex_work_processfifostore) {
		printk(KERN_DEBUG "work PROCESSFIFOSTORE called\n");

		while (fifostorepos > 0) { // processing all items in the fifo store
			printk(KERN_DEBUG "%d entries in fifo store\n", fifostorepos);

			printk(KERN_DEBUG "sending %d bytes\n", fifostore[0].size);
			BEAGLEBONE_LED3ON;
			spi_write_reg_burst(SPIDEVDATAREG, fifostore[0].data, fifostore[0].size);
			BEAGLEBONE_LED3OFF;

			// left shifting the FIFO store
			for (i = 1; i < FIFOSTORESIZE; i++) {
				for (j = 0; j < fifostore[i].size; j++)
					fifostore[i-1].data[j] = fifostore[i].data[j];
				fifostore[i-1].size = fifostore[i].size;
			}
			fifostorepos--;
		}
	}

	// this work reads some data from the SPI device to the ringbuffer
	if (work == &spikernmodex_work_read) {
		printk(KERN_DEBUG "work READ called, ringbuf %.2x\n", ringbufferpos);

		memset(&ringbuffer[ringbufferpos].data, 0, RINGBUFFERDATASIZE);
		ringbuffer[ringbufferpos].used = 1;

		BEAGLEBONE_LED4ON;
		spi_read_reg_burst(SPIDEVDATAREG, ringbuffer[ringbufferpos].data, RINGBUFFERDATASIZE);
		ringbuffer[ringbufferpos].size = RINGBUFFERDATASIZE;
		BEAGLEBONE_LED4ON;

		printk(KERN_DEBUG "read stopped, ringbuf %.2x\n", ringbufferpos);
		ringbuffer[ringbufferpos].completed = 1;

		ringbufferpos++;
		if (ringbufferpos == RINGBUFFERSIZE)
			ringbufferpos = 0;
	}

	printk(KERN_DEBUG "work exit\n");
}

//// INTERUPT HANDLE FUNCTION CHO GPIO
// đoán là khi có thiết bị mới cắm vào
// this function gets called when an interrupt happens for the registered GPIO pins
irqreturn_t spikernmodex_interrupt_handler(int irq, void *dev_id)
{
	enum { falling, rising } type;

	printk(KERN_DEBUG "interrupt received (irq: %d)\n", irq);

	if (irq == gpio_to_irq(BEAGLEBONE_GPIO19)) {
		type = BEAGLEBONE_GPIO19_LOW ? falling : rising;

		if (type == rising) {
			printk(KERN_DEBUG "rising GPIO19 interrupt received, queueing work READ\n");
			queue_work(spikernmodex_workqueue, &spikernmodex_work_read);
			return IRQ_HANDLED;
		}
	}

	return IRQ_HANDLED;
}

// đơn giản thôi, clear buffer đi
static void spikernmodex_clearringbuffer(void)
{
	int i;

	for (i = 0; i < RINGBUFFERSIZE; i++)
		ringbuffer[i].completed = ringbuffer[i].used = 0;
}

//// USER ĐỌC SYSTEM FILE
// chắc là cái này gán cho 1 quá trình đọc khi file gì đó
// this functions gets called when the user reads the sysfs file "somereg"
static ssize_t spikernmodex_somereg_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	// outputting the value of register "SOMEREG"
	return sprintf(buf, "%x\n", spi_read_reg(SOMEREG));
}

// đấy như trên cái này gàn cho quá trình ghi ra file system
// ghi file sysfs tương ứng với gửi spi data
// this function gets called when the user writes the sysfs file "somereg"
static ssize_t spikernmodex_somereg_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int val;
	sscanf(buf, "%x", &val);
	spi_write_reg(SOMEREG, (uint8_t)val);
	printk(KERN_DEBUG "stored %.2x to register SOMEREG\n", (uint8_t)val);
	return count;
}

// this function gets called when the user writes the sysfs file "clearringbuffer"
static ssize_t spikernmodex_clearringbuffer_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	spikernmodex_clearringbuffer();
	printk(KERN_DEBUG "ringbuffer cleared.\n");
	return count;
}

// these two functions are called when the user reads the sysfs files "gpio19state" and "gpio21state"
static ssize_t spikernmodex_gpio19state_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", (BEAGLEBONE_GPIO19_LOW ? 0 : 1));
}
static ssize_t spikernmodex_gpio21state_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", (BEAGLEBONE_GPIO21_LOW ? 0 : 1));
}

//<FIX ERR> negative width in bit-field ‘<anonymous>’
//CHANGE 0666 -> 0664
static struct kobj_attribute store_attribute = __ATTR(data, 0664, spikernmodex_store_show, spikernmodex_store_store);
static struct kobj_attribute somereg_attribute = __ATTR(addr, 0664, spikernmodex_somereg_show, spikernmodex_somereg_store);
static struct kobj_attribute clearringbuffer_attribute = __ATTR(clearringbuffer, 0664, NULL, spikernmodex_clearringbuffer_store);
static struct kobj_attribute gpio19state_attribute = __ATTR(gpio19state, 0664, spikernmodex_gpio19state_show, NULL);
static struct kobj_attribute gpio21state_attribute = __ATTR(gpio21state, 0664, spikernmodex_gpio21state_show, NULL);

// a group of attributes so that we can create and destroy them all at once
static struct attribute *attrs[] = {
	&store_attribute.attr,
	&somereg_attribute.attr,
	&clearringbuffer_attribute.attr,
	&gpio19state_attribute.attr,
	&gpio21state_attribute.attr,
	NULL, // need to NULL terminate the list of attributes
};





////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////khai báo structure spi driver để khởi tạo/////////////////////////////////////////////////////
///Bao gồm driver name và hàm được gọi khi probing and removing ////////////////////////////////////////////////////

static struct attribute_group attr_group = { .attrs = attrs };

static int spikernmodex_probe(struct spi_device *spi); // prototype
static int spikernmodex_remove(struct spi_device *spi); // prototype

/// SPI DRIVER STRUCTURE
static struct spi_driver spikernmodex_driver = {
	.driver = {
		.name = "spikernmodex", // be sure to match this with the spi_board_info modalias in arch/am/mach-omap2/board-am335xevm.c
		.owner = THIS_MODULE
	},
	.probe = spikernmodex_probe,
  .remove = spikernmodex_remove
};

// Probe SPI device
// this function gets called when a matching modalias and driver name found
// The driver's init function calls pci_register_driver() which gives the kernel a list of devices it is able to service,
// along with a pointer to the probe() function. The kernel then calls the driver's probe() function once for each device.

// This probe function starts the per-device initialization: initializing hardware, allocating resources, and registering the device with the kernel as a block or network device or whatever it is.
// tóm lại là gọi để init hardware, phân bố cái resoures và đăng ký device file
static int spikernmodex_probe(struct spi_device *spi)
{
	int err;

	printk(KERN_DEBUG "spikernmodex_probe() called.\n");

	spikernmodex_kobj = kobject_create_and_add("spikernmodex", kernel_kobj);
	if (!spikernmodex_kobj) {
		printk(KERN_ERR "unable to create sysfs object\n");
		return -ENOMEM;
	}

	// create the files associated with this kobject
	err = sysfs_create_group(spikernmodex_kobj, &attr_group);
	if (err) {
		kobject_put(spikernmodex_kobj);
		printk(KERN_ERR "unable to create sysfs files\n");
		return -ENOMEM;
	}

	// initalizing SPI interface
	myspi = spi;
	myspi->max_speed_hz = 5000000; // get this from your SPI device's datasheet
	spi_setup(myspi);

	// initializing device
	spi_cmd(SOMEKINDOFRESETCOMMAND);
	mdelay(100);

	// initializing workqueue
	spikernmodex_workqueue = create_singlethread_workqueue("spikernmodex_workqueue");
	INIT_WORK(&spikernmodex_work_processfifostore, spikernmodex_workqueue_handler);
	INIT_WORK(&spikernmodex_work_read, spikernmodex_workqueue_handler);

	// initializing interrupt for GPIO19
	// we need to set the interrupt for both falling and rising trigger as the (dynamically configurable) GPIO function can be triggered on both edges
	err = request_irq(gpio_to_irq(BEAGLEBONE_GPIO19), spikernmodex_interrupt_handler, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, GPIO19_INT_NAME, NULL);
	if (err) {
		printk(KERN_ERR "request IRQ error: GPIO19 already claimed or allocation failed!\n");
		return(-EIO);
	}

	spikernmodex_clearringbuffer();

	printk(KERN_INFO "Example SPI driver by Nonoo (www.nonoo.hu) loaded.\n");

	return err;
};

// this function gets called when our example SPI driver gets removed with spi_unregister_driver()
// tóm lại là gọi để free hardware, free resource
static int spikernmodex_remove(struct spi_device *spi)
{
	printk(KERN_DEBUG "spikernmodex_remove() called.\n");

	// freeing used interrupts
	free_irq(gpio_to_irq(BEAGLEBONE_GPIO19), NULL);

	// resetting device
	spi_cmd(SOMEKINDOFRESETCOMMAND);

	// destroying the sysfs structure
	kobject_put(spikernmodex_kobj);

	// destroying the workqueue
	flush_workqueue(spikernmodex_workqueue);
	destroy_workqueue(spikernmodex_workqueue);

	return 0;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////init and exit module function /////////////////////////////////////////////////////////////

// this gets called on module init
static int __init spikernmodex_init(void)
{
	int error;
	printk(KERN_INFO "Loading linux kernel SPI driver ...\n");
	// registering SPI driver, this will call spikernmodex_probe()
	error = spi_register_driver(&spikernmodex_driver);
	if (error < 0) {
		printk(KERN_ERR "spi_register_driver() failed %d\n", error);
		return error;
	}
	return 0;
}

// this gets called when module is getting unloaded
static void __exit spikernmodex_exit(void)
{
	// unregistering SPI driver, this will call cc1101_remove()
	spi_unregister_driver(&spikernmodex_driver);
	printk(KERN_INFO " SPI driver removed !!\n");
}

module_init(spikernmodex_init);
module_exit(spikernmodex_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tuan Nguyen");
MODULE_DESCRIPTION("SPI linux kernel driver for raspberry spi");
MODULE_VERSION("1.0");
