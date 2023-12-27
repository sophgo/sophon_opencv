#ifndef __U_SYS_UAPI_H__
#define __U_SYS_UAPI_H__

#ifdef __cplusplus
	extern "C" {
#endif

#include <linux/version.h>


/* chip version list */
enum ENUM_CHIP_VERSION {
	E_CHIPVERSION_U01 = 1,	//1
	E_CHIPVERSION_U02,	//2
};

/* chip power on reason list */
enum ENUM_CHIP_PWR_ON_REASON {
	E_CHIP_PWR_ON_COLDBOOT = 1,	//1
	E_CHIP_PWR_ON_WDT,	//2
	E_CHIP_PWR_ON_SUSPEND,	//3
	E_CHIP_PWR_ON_WARM_RST,	//4
};

struct sys_cdma_copy {
	__u64 u64PhyDst;
	__u64 u64PhySrc;
	__u32 u32Len;
};


#define SYS_IOC_MAGIC		'S'

#define SYS_IOC_SET_INIT				_IOW(SYS_IOC_MAGIC, 0x1, unsigned int)
#define SYS_IOC_GET_INIT				_IOR(SYS_IOC_MAGIC, 0x2, unsigned int)
#define SYS_IOC_SET_VIVPSSMODE			_IOW(SYS_IOC_MAGIC, 0x3, VI_VPSS_MODE_S)
#define SYS_IOC_GET_VIVPSSMODE			_IOR(SYS_IOC_MAGIC, 0x4, VI_VPSS_MODE_S)
#define SYS_IOC_READ_CHIP_ID			_IOR(SYS_IOC_MAGIC, 0x5, unsigned int)
#define SYS_IOC_READ_CHIP_VERSION		_IOR(SYS_IOC_MAGIC, 0x6, unsigned int)
#define SYS_IOC_READ_CHIP_PWR_ON_REASON	_IOR(SYS_IOC_MAGIC, 0x7, unsigned int)
#define SYS_IOC_CDMA_COPY				_IOW(SYS_IOC_MAGIC, 0x8, struct sys_cdma_copy)
#define SYS_IOC_CDMA_COPY2D				_IOW(SYS_IOC_MAGIC, 0x9, CVI_CDMA_2D_S)

#ifdef __cplusplus
	}
#endif

#endif /* __U_SYS_UAPI_H__ */