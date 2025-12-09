#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/of_device.h>

/* Misc */
#define POE_PORT_REG(base, port, offset) (base + (port * offset))
#define POE_FW_CRC_MAX_TIME 10

/* FW Loading */
#define I2C_BLOCK_SIZE 32
#define FW_BLOCK_SIZE 30

#define FIRMWARE_59111_1 "brcm/bcm59111.bin"

/* Defined PoE register space */
#define POE_MAX_REG 0x58

/* Number of PoE ports on chip */
#define POE_MAX_PORT  0x04

#define POE_PORT0     0x00
#define POE_PORT1     0x01
#define POE_PORT2     0x02
#define POE_PORT3     0x03
#define POE_ALL_PORTS 0x0F

/* Common register stuff */
#define POE_SINGLE_BIT 0x1
#define POE_NO_OFFSET 0x0

/* Event Registers */
#define POE_SUPENV_REG 0x0A //RO
#define POE_SUPENV_UVLO_MAIN_OFF 4
#define POE_SUPENV_UVLO_VCC_OFF 5
#define POE_SUPENV_FET_OFF 6
#define POE_SUPENV_TEMP_OFF 7

/* Status Registers */
#define POE_STATPWR_REG 0x10 //RO
#define POE_STATPWR_PORT_EN_OFF 0
#define POE_STATPWR_PORT_GOOD_OFF 4

#define POE_STATPIN_REG 0x11 //RO
#define POE_STATPIN_AUTO(_reg, _port)    (_reg & 0x1)
#define POE_STATPIN_ID(_reg, _port)      (_reg & (0x3 << 2))

#define POE_STATP_BASE   0x0C //RO
#define POE_STATP_OFFSET 0x01
#define POE_STATP_DETECT_BITS 0x7
#define POE_STATP_DETECT_OFFSET 0
#define POE_STATP_CLASS_BITS  0x7
#define POE_STATP_CLASS_OFFSET  4
#define POE_STATP_REG(port) \
	(POE_PORT_REG(POE_STATP_BASE, port, POE_STATP_OFFSET))

static char *class[] = { "unknown", "1",	"2", "3",
			 "4",	    "reserved", "0", "overcurrent" };
static char *mode[] = { "UNK", "AT", "AT", "AT", "AT", "RESV", "AF", "OC" };

#define DETECT_UNKNOWN   0x0
#define DETECT_SHORT     0x1
#define DETECT_CPD_HIGH  0x2
#define DETECT_RSIG_LOW  0x3
#define DETECT_GOOD      0x4
#define DETECT_RSIG_HIGH 0x5
#define DETECT_OPEN      0x6

char *detect_state[] = { "unknown",  "short", "cpd_high",
			 "rsig_low", "good",  "open" };

#define CLASS_UNKNOWN 0x0
#define CLASS_CLASS_1 0x1
#define CLASS_CLASS_2 0x2
#define CLASS_CLASS_3 0x3
#define CLASS_CLASS_4 0x4
#define CLASS_CLASS_0 0x6
#define CLASS_OVERCUR 0x7

/* Global configuration registers */
#define POE_OP_MODE_REG  0x12 //RW
#define POE_OP_MODE_OFF  0
#define POE_OP_MODE_BITS 0x3

#define OP_MODE_SHUTDOWN 0x0
#define OP_MODE_MANUAL   0x1
#define OP_MODE_SEMI     0x2
#define OP_MODE_AUTO     0x3
#define OP_MODE_CLR(opmod, port) (opmod & ~(0x3 << (port * 2)))
#define OP_MODE_SET(opmod, port) (opmod << (port * 2))
#define OP_MODE_CLR_SET(reg, opmod, port) \
	(OP_MODE_CLR(reg, port) | OP_MODE_SET(opmod, port))
#define OP_MODE_SET_ALL(opmod)                                           \
	(OP_MODE_SET(opmod, POE_PORT3) | OP_MODE_SET(opmod, POE_PORT2) | \
	 OP_MODE_SET(opmod, POE_PORT1) | OP_MODE_SET(opmod, POE_PORT0))

#define OP_MODE_DEFAULT OP_MODE_AUTO

static const u8 OP_MODES_SET_ALL[] = {
	OP_MODE_SET_ALL(OP_MODE_SHUTDOWN),
	OP_MODE_SET_ALL(OP_MODE_MANUAL),
	OP_MODE_SET_ALL(OP_MODE_SEMI),
	OP_MODE_SET_ALL(OP_MODE_AUTO),
};

static u8 op_mode = OP_MODE_DEFAULT;

#define POE_DISC_SENSE_REG 0x13 //RW
#define DISC_DC_BASE 0x0
#define DISC_AC_BASE 0x4
#define DISC_ENABLE_DC(ports)   (ports << DISC_DC_BASE)
#define DISC_ENABLE_AC(ports)   (ports << DISC_AC_BASE)
#define DISC_ENABLE_ACDC(ports) (DISC_ENABLE_AC(ports) | DISC_ENABLE_DC(ports))

#define POE_DET_CLASS_REG 0x14
#define DETECT_BASE 0x0
#define CLASS_BASE  0x4
#define DETECT_ENABLE(ports)    (ports << DETECT_BASE)
#define CLASS_ENABLE(ports)     (ports << CLASS_BASE)
#define DET_CLASS_ENABLE(ports) (CLASS_ENABLE(ports) | DETECT_ENABLE(ports))

/* Debug Registers */
#define POE_CHIP_ID_REG  0x1B //RO
#define CHIP_MFG_ID(val) (val & 0xf0 >> 4)
#define CHIP_IC_ID(val)  (val & 0xf)

/* PoE Port Parametric Measurements */
#define POE_PARAM_AMP_BASE_REG   0x30 //RO
#define POE_PARAM_VOLT_BASE_REG  0x32 //RO
#define POE_PARAM_OFFSET         0x04

#define POE_PARAM_REG(base, port) (POE_PORT_REG(base, port, POE_PARAM_OFFSET))
#define POE_PARAM_VOLT_REG(port)  (POE_PARAM_REG(POE_PARAM_VOLT_BASE_REG, port))
#define POE_PARAM_AMP_REG(port)   (POE_PARAM_REG(POE_PARAM_AMP_BASE_REG, port))
#define POE_STEP_VOLTAGE 5835 //(uV)
#define POE_STEP_CURRENT 122 //(uA) technically 122.07
#define POE_POWER(_vreg, _creg)                \
	(((_vreg * POE_STEP_VOLTAGE) / 1000) * \
	 ((_creg * POE_STEP_CURRENT) / 1000) / \
	 (1000)) //uA * uV = 1 trillionth of a Watt

/* PoE High Power Features registers */
#define POE_HP_ENABLE_REG     0x44 //RW
#define HP_ENABLE(ports)      (ports)

//Controls high power mode
#define POE_HP_MODE_BASE_REG   0x46 //RW
#define HP_MODE_PING_PONG_CL   0x0
#define HP_MODE_LEGACY_CL      0x1
#define HP_MODE_ENABLE_ALL (1 << HP_MODE_PING_PONG_CL | 1 << HP_MODE_LEGACY_CL)

//Sets current limit
#define POE_HP_CUT_BASE_REG 0x47 //RW
#define HP_CUT_RNG  0x6
#define HP_CUT_RDIS 0x7 //Always set to 1
#define HP_CUT_BASE_CUR 18.75 //(mA)
#define HP_CUT_LIMIT(mv, cut) (mv / (HP_CUT_BASE_CUR * (cut + 1)))
#define HP_CUT_SET(mv, cut) \
	(HP_CUT_LIMIT(mv, cut) | cut << 0x6 | 0x1 << HP_CUT_RDIS)
#define HP_CUT_GET(count, cut) (HP_CUT_BASE_CUR * (cut + 1) * count)

//Current
#define POE_HP_LIM_BASE_REG    0x48 //RW
#define POE_HP_STATUS_BASE_REG 0x49 //RO
#define POE_HP_FEAT_OFFSET     0x04

#define POE_HP_REG(base, port) (POE_PORT_REG(base, port, POE_HP_FEAT_OFFSET))
#define POE_HP_MODE_REG(port) (POE_HP_REG(POE_HP_MODE_BASE_REG, port))
#define POE_HP_CUT_REG(port) (POE_HP_REG(POE_HP_CUT_BASE_REG, port))
#define POE_HP_LIM_REG(port) (POE_HP_REG(POE_HP_LIM_BASE_REG, port))
#define POE_HP_STATUS_REG(port) (POE_HP_REG(POE_HP_STATUS_BASE_REG, port))

/* Available counters on PoE ports */
#define POE_COUNT_INV_SIG_BASE_REG      0x60 //COR RO
#define POE_COUNT_POWER_DENIED_BASE_REG 0x61 //COR RO
#define POE_COUNT_OVER_SHORT_BASE_REG   0x62 //COR RO
#define POE_COUNT_MPS_BASE_REG          0x63 //COR RO
#define POE_COUNT_OFFSET                0x04

#define POE_COUNT_REG(reg, port) (POE_PORT_REG(base, port, POE_COUNT_OFFSET))
#define POE_COUNT_INV_SIG_REG(port) \
	(POE_COUNT_REG(POE_COUNT_INV_SIG_BASE_REG, port))
#define POE_COUNT_POWER_DENIED_REG(port) \
	(POE_COUNT_REG(POE_COUNT_POWER_DENIED_REG, port))
#define POE_COUNT_POWER_DENIED_BITS 0xff
#define POE_COUNT_OVER_SHORT_REG(port) \
	(POE_COUNT_REG(POE_COUNT_OVER_SHORT_BASE_REG, port))
#define POE_COUNT_MPS_REG(port) (POE_COUNT_REG(POE_COUNT_MPS_BASE_REG, port))

/* Firmware loading registers */
#define POE_FW_LOAD_CTL_REG 0x70
#define POE_FW_LOAD_START   0x80
#define POE_FW_LOAD_STOP    0x40

#define POE_FW_MSB_REG  0x71
#define POE_FW_LSB_REG  0x72
#define POE_FW_DATA_REG 0x73

#define POE_FW_CRC_REG  0x75
#define POE_FW_CRC_GOOD 0xAA
#define POE_FW_CRC_WAIT 0x55
#define POE_FW_CRC_FAIL 0xFF

#define POE_ATTR_RO(_name, _func) __ATTR(_name, 0444, _func, NULL)
#define POE_ATTR_WO(_name, _func) __ATTR(_name, 0220, NULL, _func)
#define POE_ATTR_RW(_name, _show, _store) __ATTR(_name, 0664, _show, _store)

#define POE_ATTR_RO_AUTO(_name) POE_ATTR_RO(_name, _name##_show)
#define POE_ATTR_WO_AUTO(_name) POE_ATTR_WO(_name, _name##_store)
#define POE_ATTR_RW_AUTO(_name) POE_ATTR_RW(_name, _name##_show, _name##_store)

#define POE_ATTR_RO_FUNC(_name, _reg, _bits, _bitoff, _pstr, _type)            \
	static ssize_t _name##_show(struct _type *dev,                         \
				    struct _type##_attribute *attr, char *buf) \
	{                                                                      \
		s32 rd = (_reg >= 0 ? BCM_READ(poe.client, _reg) : -1);        \
		int idx = ((rd >> _bitoff) & _bits);                           \
		return _pstr;                                                  \
	}

#define _POE_PRINT(_str, _args) scnprintf(buf, PAGE_SIZE, _str, _args)
#define POE_PRINT_STR(_strs)                                                   \
	_POE_PRINT("%s\n", (rd < 0 ? "ERR" :                                   \
				     ((idx < ARRAY_SIZE(_strs)) ? _strs[idx] : \
								  "ERR")))
#define POE_PRINT_BITS _POE_PRINT("%x\n", (rd < 0 ? -1 : idx))
#define POE_PRINT_MATCH(_mval) \
	_POE_PRINT("%d\n", (rd < 0 ? -1 : (_mval == idx)));

#define DEV_RD(_name, _reg, _regoff, _bits, _bitoff)                     \
	POE_ATTR_RO_FUNC(_name, (_reg + (dev->devt * _regoff)), (_bits), \
			 (_bitoff), POE_PRINT_BITS, device)
#define DEV_SHARED_RD(_name, _reg, _regoff, _bits, _bitoff)              \
	POE_ATTR_RO_FUNC(_name, (_reg + (dev->devt * _regoff)), (_bits), \
			 (_bitoff + (dev->devt)), POE_PRINT_BITS, device)
#define DEV_MATCH(_name, _reg, _regoff, _bits, _bitoff, _mval)           \
	POE_ATTR_RO_FUNC(_name, (_reg + (dev->devt * _regoff)), (_bits), \
			 (_bitoff), POE_PRINT_MATCH(_mval), device)
#define DEV_SHARED_MATCH(_name, _reg, _regoff, _bits, _bitoff, _mval)     \
	POE_ATTR_RO_FUNC(_name, (_reg + (dev->devt * _regoff)), (_bits),  \
			 (_bitoff + (dev->devt)), POE_PRINT_MATCH(_mval), \
			 device)
#define DEV_STR_RD(_name, _reg, _regoff, _bits, _bitoff)                 \
	POE_ATTR_RO_FUNC(_name, (_reg + (dev->devt * _regoff)), (_bits), \
			 (_bitoff), POE_PRINT_STR(_name), device)

#define BCM_WRITE_BLOCK(cl, addr, len, val) \
	i2c_smbus_write_block_data(cl, addr, len, val)
#define BCM_READ_WORD(cl, addr) i2c_smbus_read_word_data(cl, addr)
#define BCM_READ(cl, addr) i2c_smbus_read_byte_data(cl, addr)
#define BCM_WRITE(cl, addr, val) i2c_smbus_write_byte_data(cl, addr, val)

struct bcm_poe_dev {
	struct device *ports[POE_MAX_PORT];
	uint32_t last_joule_time[POE_MAX_PORT];
	const struct i2c_client *client;
	struct class *class;
	struct device *pdev;
	struct device *port_pdev;
	long reg_op;
	long reg_arg;
	uint32_t nports;
	uint32_t port_base;
	uint32_t max_power_mw;
};

static struct bcm_poe_dev poe;

enum fail {
	EFW = 1,
	ECLASS,
	EDEVICE,
	EDEVICEF,
	EPORT,
	EPORTF,
	EREAD,
	EWRITE,
	EINIT,
	ECRC,
	ENOPORT,
	ENOBASE,
	ENOPOWER,
};

static const struct firmware *bcm_poe_fw;

static ssize_t reg_store(struct device *cl, struct device_attribute *cl_attr,
			 const char *buf, size_t count)
{
	long val;

	if (!kstrtol(buf, 16, &val)) {
		if (!strcmp(cl_attr->attr.name, "reg_arg"))
			BCM_WRITE(poe.client, poe.reg_op, val);
		else if (!strcmp(cl_attr->attr.name, "reg"))
			poe.reg_op = val;
	}

	return count;
}

static ssize_t reg_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "Register 0x%lx Value 0x%x\n",
			 poe.reg_op, BCM_READ(poe.client, poe.reg_op));
}

static ssize_t enabled_store(struct device *dev,
			     struct device_attribute *cl_attr, const char *buf,
			     size_t count)
{
	long val;
	s32 rd;

	if (!kstrtol(buf, 16, &val)) {
		rd = BCM_READ(poe.client, POE_OP_MODE_REG);
		rd = OP_MODE_CLR_SET(
			rd, (val ? OP_MODE_AUTO : OP_MODE_SHUTDOWN), dev->devt);
		BCM_WRITE(poe.client, POE_OP_MODE_REG, rd);
	}

	return count;
}

static u64 calculate_port_power_usage(int port)
{
	u32 volt, amp;
	u64 ret;

	volt = BCM_READ_WORD(poe.client, POE_PARAM_VOLT_REG(port));
	amp = BCM_READ_WORD(poe.client, POE_PARAM_AMP_REG(port));

	if (volt < 0 || amp < 0)
		ret = -EREAD;
	else
		ret = POE_POWER(volt, amp);

	return ret;
}

static ssize_t millijoules_show(struct device *dev,
				struct device_attribute *dev_attr, char *buf)
{
	s64 watts = calculate_port_power_usage(dev->devt);
	uint32_t t = (jiffies - poe.last_joule_time[dev->devt]);
	poe.last_joule_time[dev->devt] = jiffies;
	return scnprintf(buf, PAGE_SIZE, "%llu\n", watts * t);
}

static ssize_t power_show(struct device *dev, struct device_attribute *dev_attr,
			  char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%llu\n",
			 calculate_port_power_usage(dev->devt));
}

static ssize_t power_budget_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", poe.max_power_mw);
}

static void remove_sysfs_dirs(void)
{
	int i;
	for (i = 0; i < poe.nports && poe.ports[i]; i++)
		device_unregister(poe.ports[i]);

	if (poe.pdev)
		device_unregister(poe.pdev);

	if (poe.class)
		class_unregister(poe.class);
}

static int bcm_fw_init(struct device *dev)
{
	int fail = 0;
	u32 i, j;
	u8 block[I2C_BLOCK_SIZE];
	unsigned long start;

	if (BCM_READ(poe.client, POE_FW_CRC_REG) == POE_FW_CRC_GOOD)
		goto fw_loaded;

	fail = BCM_WRITE(poe.client, POE_FW_LOAD_CTL_REG, POE_FW_LOAD_START);

	if (fail < 0)
		goto init_fail;

	fail = request_firmware(&bcm_poe_fw, FIRMWARE_59111_1, dev);

	if (fail != 0)
		goto init_fail;

	for (i = 0; i < bcm_poe_fw->size && fail >= 0; i += FW_BLOCK_SIZE) {
		block[0] = ((i & 0xff00) >> 8);
		block[1] = (i & 0xff);
		for (j = 0; (j < FW_BLOCK_SIZE) && (i + j) < bcm_poe_fw->size;
		     j++)
			block[j + 2] = bcm_poe_fw->data[i + j];
		fail = BCM_WRITE_BLOCK(poe.client, POE_FW_LOAD_CTL_REG, (j + 2),
				       block);
	}

	if (fail < 0)
		goto init_fail;

	fail = BCM_WRITE(poe.client, POE_FW_LOAD_CTL_REG, POE_FW_LOAD_STOP);

	if (fail < 0)
		goto init_fail;

	start = ktime_get_real_seconds();
	while (BCM_READ(poe.client, POE_FW_CRC_REG) != POE_FW_CRC_GOOD &&
	       ktime_get_real_seconds() - start < POE_FW_CRC_MAX_TIME) {
		msleep(100);
	};

	if (BCM_READ(poe.client, POE_FW_CRC_REG) != POE_FW_CRC_GOOD) {
		fail = -ECRC;
		goto init_fail;
	}

fw_loaded:
	//Enable disconnect sensing
	BCM_WRITE(poe.client, POE_DISC_SENSE_REG,
		  DISC_ENABLE_DC(POE_ALL_PORTS));
	//Classify/detect on ports 0/1/2/3 (only used are 0/1)
	BCM_WRITE(poe.client, POE_DET_CLASS_REG,
		  DET_CLASS_ENABLE(POE_ALL_PORTS));

	if (op_mode == OP_MODE_DEFAULT)
		BCM_WRITE(poe.client, POE_OP_MODE_REG,
			  OP_MODE_SET_ALL(OP_MODE_DEFAULT));

init_fail:
	release_firmware(bcm_poe_fw);
	return fail;
}

DEV_RD(denied, POE_COUNT_POWER_DENIED_BASE_REG, POE_COUNT_OFFSET,
       POE_COUNT_POWER_DENIED_BITS, POE_NO_OFFSET);
DEV_RD(uvlo_main, POE_SUPENV_REG, POE_NO_OFFSET, POE_SINGLE_BIT,
       POE_SUPENV_UVLO_MAIN_OFF);
DEV_RD(uvlo_vcc, POE_SUPENV_REG, POE_NO_OFFSET, POE_SINGLE_BIT,
       POE_SUPENV_UVLO_VCC_OFF);
DEV_RD(failed_fet, POE_SUPENV_REG, POE_NO_OFFSET, POE_SINGLE_BIT,
       POE_SUPENV_FET_OFF);
DEV_RD(failed_temp, POE_SUPENV_REG, POE_NO_OFFSET, POE_SINGLE_BIT,
       POE_SUPENV_TEMP_OFF);

DEV_STR_RD(class, POE_STATP_BASE, POE_STATP_OFFSET, POE_STATP_CLASS_BITS,
	   POE_STATP_CLASS_OFFSET);
DEV_STR_RD(mode, POE_STATP_BASE, POE_STATP_OFFSET, POE_STATP_CLASS_BITS,
	   POE_STATP_CLASS_OFFSET);

DEV_MATCH(overload, POE_STATP_BASE, POE_STATP_OFFSET, POE_STATP_DETECT_BITS,
	  POE_STATP_DETECT_OFFSET, DETECT_RSIG_HIGH);
DEV_MATCH(underload, POE_STATP_BASE, POE_STATP_OFFSET, POE_STATP_DETECT_BITS,
	  POE_STATP_DETECT_OFFSET, DETECT_RSIG_LOW);

DEV_SHARED_RD(enabled, POE_STATPWR_REG, POE_NO_OFFSET, POE_SINGLE_BIT,
	      POE_STATPWR_PORT_EN_OFF);
DEV_SHARED_RD(status, POE_STATPWR_REG, POE_NO_OFFSET, POE_SINGLE_BIT,
	      POE_STATPWR_PORT_EN_OFF);

static struct device_attribute port_attrs[] = {
	POE_ATTR_RO_AUTO(class),
	POE_ATTR_RO_AUTO(denied),
	POE_ATTR_RO_AUTO(millijoules),
	POE_ATTR_RO_AUTO(mode),
	POE_ATTR_RO_AUTO(overload),
	POE_ATTR_RO_AUTO(power),
	POE_ATTR_RW_AUTO(enabled),
	POE_ATTR_RO_AUTO(status),
	POE_ATTR_RO_AUTO(underload),
};

static struct device_attribute dev_attrs[] = {
	POE_ATTR_RO_AUTO(failed_fet),
	POE_ATTR_RO_AUTO(failed_temp),
	POE_ATTR_RO_AUTO(uvlo_main),
	POE_ATTR_RO_AUTO(uvlo_vcc),
	POE_ATTR_RO_AUTO(power_budget),
	POE_ATTR_RW(reg, reg_show, reg_store),
	POE_ATTR_WO(reg_arg, reg_store),
};

static void remove_sysfs_links(struct kobject *dev_kobj)
{
	sysfs_remove_link(dev_kobj, "subsystem");
	sysfs_remove_link(dev_kobj, "power");
	sysfs_remove_link(dev_kobj, "uevent");
	sysfs_remove_link(dev_kobj, "device");
}

static void remove_device_files(void)
{
	int i, j;

	for (i = 0; i < ARRAY_SIZE(dev_attrs); i++)
		device_remove_file(poe.pdev, &(dev_attrs[i]));

	for (i = 0; i < poe.nports; i++) {
		if (poe.ports[i]) {
			for (j = 0; j < ARRAY_SIZE(port_attrs); j++)
				device_remove_file(poe.ports[i],
						   &(port_attrs[j]));
		}
	}
}

static int get_device_params(void)
{
	poe.nports = -1;
	poe.port_base = -1;
	poe.max_power_mw = -1;

#define PARG(_name, _dst) \
	of_property_read_u32(poe.client->dev.of_node, _name, _dst)
	if (PARG("ports", &(poe.nports)) || poe.nports < 0 ||
	    poe.nports > POE_MAX_PORT)
		return -ENOPORT;
	else if (PARG("port_base", &(poe.port_base)) || poe.port_base < 0)
		return -ENOBASE;
	else if (PARG("max_power_mw", &(poe.max_power_mw)) ||
		 poe.max_power_mw < 0)
		return -ENOPOWER;
#undef PARG

	return 0;
}

static int bcm_poe_probe(struct i2c_client *cl)
{
	int i, j, ret = 0;

	poe.client = cl;
	poe.class = class_create("poe");

	if (!poe.class) {
		ret = -ECLASS;
		goto probe_failed;
	}

	poe.pdev = device_create(poe.class, NULL, 0, NULL, "poe0");

	if (!poe.pdev) {
		ret = -EDEVICE;
		goto probe_failed;
	}

	ret = get_device_params();
	if (ret)
		goto probe_failed;

	if (bcm_fw_init(poe.pdev)) {
		ret = -EFW;
		goto probe_failed;
	}

	poe.port_pdev =
		device_create(poe.class, poe.pdev, 0, NULL, "sys_ports");

	if (!poe.port_pdev) {
		ret = -EDEVICE;
		goto probe_failed;
	}

	for (i = 0; i < ARRAY_SIZE(dev_attrs); i++) {
		if (device_create_file(poe.pdev, &(dev_attrs[i]))) {
			ret = -EDEVICEF;
			goto probe_failed;
		}
	}

	remove_sysfs_links(&(poe.pdev->kobj));
	remove_sysfs_links(&(poe.port_pdev->kobj));

	for (i = 0; i < poe.nports && !ret; i++) {
		poe.ports[i] = device_create(poe.class, poe.port_pdev, i, NULL,
					     "port%d", i + poe.port_base);

		if (!poe.ports[i]) {
			ret = -EPORT;
		} else {
			remove_sysfs_links(&(poe.ports[i]->kobj));

			for (j = 0; j < ARRAY_SIZE(port_attrs) && !ret; j++) {
				if (device_create_file(poe.ports[i],
						       &(port_attrs[j])))
					ret = -EPORTF;
			}

			poe.last_joule_time[i] = jiffies;
		}
	}

probe_failed:
	if (ret < 0) {
		remove_device_files();
		remove_sysfs_dirs();
	}

	return ret;
}

static void bcm_poe_remove(struct i2c_client *client)
{
	remove_device_files();
	remove_sysfs_dirs();
}

static int param_set_mode(const char *val, const struct kernel_param *kp)
{
	int ret =
		param_set_uint_minmax(val, kp, OP_MODE_SHUTDOWN, OP_MODE_AUTO);
	if (ret == 0)
		BCM_WRITE(poe.client, POE_OP_MODE_REG,
			  OP_MODES_SET_ALL[*((unsigned int *)kp->arg)]);
	return ret;
}

static const struct kernel_param_ops param_mode = {
	.set = param_set_mode,
	.get = param_get_int,
};

module_param_cb(mode, &param_mode, &op_mode, 0644);
MODULE_PARM_DESC(
	mode, "Operation mode (0: Shutdown; 1: Manual; 2: Semi; 3: Auto)[3]");

static struct i2c_device_id bcm_poe_idtable[] = {
	{ "bcm59111", 0 },
	{ }
};

static struct of_device_id bcm_poe_of_idtable[] = {
	{ .compatible = "brcm,bcm59111" },
	{ }
};

static struct i2c_driver bcm_poe_driver = {
	.driver = {
		.name   = "bcm59111",
		.of_match_table = bcm_poe_of_idtable,
		.owner  = THIS_MODULE,
	},

	.id_table   = bcm_poe_idtable,
	.probe      = bcm_poe_probe,
	.remove = bcm_poe_remove,
};

static int __init bcm_poe_init(void)
{
	return i2c_add_driver(&bcm_poe_driver);
}

static void __exit bcm_poe_exit(void)
{
	i2c_del_driver(&bcm_poe_driver);
}

module_init(bcm_poe_init);
module_exit(bcm_poe_exit);
MODULE_DEVICE_TABLE(i2c, bcm_poe_idtable);
MODULE_DEVICE_TABLE(of, bcm_poe_of_idtable);

MODULE_AUTHOR("Timothy Passaro");
MODULE_DESCRIPTION("Driver for BCM59111");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(FIRMWARE_59111_1);
MODULE_VERSION("1.1.0");
