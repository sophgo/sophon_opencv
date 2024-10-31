#include "precomp.hpp"

#ifndef WIN32
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#endif
#define MAX_CARD_NUMS 128

namespace cv { namespace hal {

typedef enum {
  HEAP_TPU = 0,
  HEAP_VPP = 1,
  HEAP_VPU = 2,
  HEAP_MAX = 3
};

#ifndef WIN32

int getChipId(unsigned int *chipid, int id)
{
  static unsigned int hal_chipid[MAX_CARD_NUMS] = {0};
  if (hal_chipid[id] == 0) {
    bm_handle_t handle;
    id = BM_CARD_ID(id);
    int ret = bm_dev_request(&handle, id);
    if (ret != BM_SUCCESS) {
        av_log(NULL, AV_LOG_ERROR, "av 420 Create bm handle failed. ret = %d\n", ret);
        return ret;
    }
    bm_get_chipid(handle, &hal_chipid[id]);
    bm_dev_free(handle);
  }
  *chipid = hal_chipid[id];
  return 0;
}

class IONAllocator : public MatAllocator
{
private:
  int dev;

public:
  IONAllocator()
  {
    dev = open("/dev/ion", O_RDWR | O_DSYNC);
    CV_Assert(dev > 0);
  }

  UMatData* allocate(int dims, const int* sizes, int type,
             void* data0, size_t* step, AccessFlag /*flags*/, UMatUsageFlags /*usageFlags*/) const
  {
    return allocate(dims, sizes, type, data0, step, 0);
  }

  UMatData* allocate(int /*dims*/, const int* sizes, int type, void* data0, size_t* step, int id) const
  {
    step[1] = CV_ELEM_SIZE(type);
    step[0] = CV_ELEM_SIZE(type) * sizes[1];
// #ifdef HAVE_BMCV
//     bm_get_chipid(bmcv::getCard(id),&chipid);
// #endif
    //Disable 64-bit alignment for 1684x with respect to step\

    unsigned int chipid;
    if (getChipId(&chipid, id) != BM_SUCCESS)
      return NULL;
    if (CV_MAT_DEPTH(type) == CV_8U && sizes[0] > 1 && sizes[1] > 1) {
      if (chipid == 0x1684 )
        step[0] = (step[0] + 63) & (~63);
      else
        step[0] = (step[0] + 31) & (~31);
      }

    size_t total = step[0] * sizes[0];

    int fd;
    bm_uint64 addr;
    void* data = data0 ? data0 : ionMalloc(total, id, &addr, &fd);
    if (!data) return NULL;

    UMatData* u = new UMatData(this);
    if (!u) {
      CV_Error(CV_HalMemErr, "ion UMatData failed");
      return NULL;
    }

    u->data = u->origdata = static_cast<uchar*>(data);
    u->size = total;
    u->addr = addr;
    u->fd = fd;

    if (data0) {
      u->flags |= UMatData::USER_ALLOCATED;
    }
    return u;
  }

  UMatData* allocate(int total, int id) const
  {
    int fd;
    bm_uint64 addr;
    void* data = ionMalloc(total, id, &addr, &fd);
    if (!data) return NULL;

    UMatData* u = new UMatData(this);
    if (!u) {
      CV_Error(CV_HalMemErr, "ion UMatData failed");
      return NULL;
    }

    u->data = u->origdata = static_cast<uchar*>(data);
    u->size = total;
    u->addr = addr;
    u->fd = fd;
    return u;
  }

  UMatData* allocate(int total, void *data0, bm_uint64 addr0, int fd0, int id) const
  {
    int fd = fd0;
    bm_uint64 addr = fd0 >= 0 ? addr0 : 0;
    void *data = data0;

    if (fd0 >= 0 && !data0){
        data = mmap(NULL, total, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
        if (data == MAP_FAILED) {
#ifdef HAVE_BMCV
	        /* try ion allocation failed, now try bmlib mmap */
	        bm_handle_t handle = bmcv::getCard(0);
	        bm_device_mem_t dmem;

			dmem = bm_mem_from_device(addr, total);
			if (BM_SUCCESS == bm_mem_mmap_device_mem(handle, &dmem, (bm_uint64*)&data))
				fd = -1;    // no valid fd handle
			else
#endif
			{
				CV_Error(CV_HalMemErr, "mmap failed");
				return NULL;
			}
        }
    }else if (!data0)
        data = ionMalloc(total, id, &addr, &fd);
    if (!data) return NULL;

    UMatData* u = new UMatData(this);
    if (!u){
        CV_Error(CV_HalMemErr, "ion UMatData failed");
        return NULL;
    }

    u->data = u->origdata = static_cast<uchar*>(data);
    u->size = total;
    u->addr = addr;
    u->fd = fd;

    if (data0) {
        u->flags |= UMatData::USER_ALLOCATED;  // soc mode, USER_ALLOCATED DEVICE_MEM_ATTACHED is same
    }
    if (fd0 >= 0){
        u->flags |= UMatData::DEVICE_MEM_ATTACHED;
    }
    return u;
  }

  bool allocate(UMatData* u, AccessFlag /*accessFlags*/, UMatUsageFlags /*usageFlags*/) const
  {
    if(!u) return false;
    return true;
  }

  void deallocate(UMatData* u) const
  {
    if (!u) return;

    CV_Assert(u->urefcount == 0);
    CV_Assert(u->refcount == 0);
    if (!(u->flags & UMatData::USER_ALLOCATED))
    {
        if (u->flags & UMatData::DEVICE_MEM_ATTACHED){
            if (u->origdata) munmap(u->origdata, u->size);
        } else
            ionFree(u->origdata, u->size, u->fd);
        u->origdata = 0;
    }
    delete u;
  }

  void flush(UMatData* u, size_t sz) const
  {
    if (!u) return;

    ionFlush(u->data, sz ? sz : u->size);
  }

  void invalidate(UMatData* u, size_t size) const
  {
    if (!u) return;

    ionInvalidate(u->data, size ? size : u->size);
  }
private:
  int queryHeapID(ion_heap_type type, int id) const
  {
    int ret;
    struct ion_heap_query heap_query;
    struct ion_heap_data *heap_data;
    int heap_id = -1;
    const char *ion_name[4] = {"vpp", "npu", "vpp", "vpu"};

    memset(&heap_query, 0, sizeof(heap_query));
    ret = ioctl(dev, ION_IOC_HEAP_QUERY, &heap_query);
    if (ret < 0 || heap_query.cnt == 0) {
      CV_Error(CV_HalMemErr, "ion ION_IOC_HEAP_QUERY failed");
      return -1;
    }

    id = (id > heap_query.cnt)?heap_query.cnt:id;

    heap_data = (struct ion_heap_data*)calloc(heap_query.cnt, sizeof(struct ion_heap_data));
    if (heap_data == NULL) {
      //CV_Error(CV_HalMemErr, "calloc failed");
      return -1;
    }

    heap_query.heaps = (uint64_t)heap_data;
    ret = ioctl(dev, ION_IOC_HEAP_QUERY, &heap_query);
    if (ret < 0) {
      //CV_Error(CV_HalMemErr, "ion ION_IOC_HEAP_QUERY failed");
      return -1;
    }

    heap_id = heap_query.cnt;
    for(int i = 0;i < heap_query.cnt;i++) {
      if ((heap_data[i].type == type) && (strcmp(heap_data[i].name, ion_name[id]) == 0)) {
        heap_id = heap_data[i].heap_id;
        break;
      } else if ((heap_data[i].type == type) && (strcmp(heap_data[i].name, "carveout") == 0)) {
        heap_id = heap_data[i].heap_id;
        break;
      }
    }

    if (heap_id == heap_query.cnt)
      heap_id = -1;

    free(heap_data);
    return heap_id;
  }

  void* ionMalloc(size_t size, int id, bm_uint64* addr, int* fd) const
  {
    int ret;
    int heap_id;

    struct ion_allocation_data allocData;
    memset(&allocData, 0, sizeof(allocData));

    allocData.len = size;
    allocData.flags = ION_FLAG_CACHED;

    int bm_card_heap = BM_CARD_HEAP(id);
    int heap_id_arry[HEAP_MAX] = {0};
    int ion_alloc_success = 0;
    if (bm_card_heap == 0) {
        bm_card_heap = (1 << HEAP_VPP) | (1 << HEAP_VPU);
    }
    for (int i = 0;i < HEAP_MAX; i++) {
        if (ion_alloc_success) {
            break;
        }

        if((bm_card_heap & (1 << i)) == 0) {
            continue;
        }
        id = i+1;

        heap_id = queryHeapID(ION_HEAP_TYPE_CARVEOUT, id);
        if (heap_id < 0) {
          //CV_Error(CV_HalMemErr, "ion ION_IOC_HEAP_QUERY failed");
          continue;
        }

        allocData.heap_id_mask = (1 << heap_id);
        ret = ioctl(dev, ION_IOC_ALLOC, &allocData);
        if (ret < 0) {
          //CV_Error(CV_HalMemErr, "ion ION_IOC_ALLOC failed");
          continue;
        }
        ion_alloc_success = 1;
    }

    if (ion_alloc_success != 1) {
        return NULL;
    }
    void *p = mmap(NULL, size, PROT_READ|PROT_WRITE, MAP_SHARED, allocData.fd, 0);
    if (p == MAP_FAILED) {
      //CV_Error(CV_HalMemErr, "mmap failed");
      return NULL;
    }

    *addr = allocData.paddr;
    *fd = allocData.fd;
    return p;
  }

  void ionFree(void* vddr, size_t size, int fd) const
  {
    if (vddr) munmap(vddr, size);
    close(fd);
  }

  void ionFlush(void* vddr, size_t size) const
  {
    int ret;
    struct cv_ion_custom_data custom_data;
    struct cv_bitmain_cache_range cache_range;

    cache_range.start = vddr;
    cache_range.size = size;

    custom_data.cmd = ION_IOC_BITMAIN_FLUSH_RANGE;
    custom_data.arg = (ulong)&cache_range;

    ret = ioctl(dev, ION_IOC_CUSTOM, &custom_data);
    if (ret < 0) {
      CV_Error(CV_HalMemErr, "ion ION_IOC_CUSTOM failed");
    }
  }
  void ionInvalidate(void* vaddr, size_t size) const
  {
    int ret;
    struct cv_ion_custom_data custom_data;
    struct cv_bitmain_cache_range cache_range;

    cache_range.start = vaddr;
    cache_range.size = size;

    custom_data.cmd = ION_IOC_BITMAIN_INVALIDATE_RANGE;
    custom_data.arg = (ulong)&cache_range;

    ret = ioctl(dev, ION_IOC_CUSTOM, &custom_data);
    if (ret < 0) {
        CV_Error(CV_HalMemErr, "ioctl ION_IOC_CUSTOM failed");
    }
  }
};
#endif

//////////////////////////////////////////////////////////////////////////////////

#ifdef HAVE_BMCV

class PCIAllocator : public MatAllocator
{
public:
  UMatData* allocate(int dims, const int* sizes, int type,
             void* data0, size_t* step, AccessFlag /*flags*/, UMatUsageFlags /*usageFlags*/) const
  {
    return allocate(dims, sizes, type, data0, step, 0);
  }

  UMatData* allocate(int dims, const int* sizes, int type, void* data0, size_t* step, int id) const
  {
    int i = dims;
    while (i > 0) {
      if (i == dims)
        step[i-1] = CV_ELEM_SIZE(type);
      else
        step[i-1] = step[i] * sizes[i];
      i--;
    }

    size_t total = step[0] * sizes[0];
    bm_handle_t hid = bmcv::getCard(id);

    bm_uint64 addr;
    bm_device_mem_t mem;
    memset(&mem, 0, sizeof(bm_device_mem_t));
    if (!data0)
        if (!pciDeviceMalloc(total, hid, id, &mem, &addr)) return NULL;

    void* data = data0 ? data0 : pciSystemMalloc(total);
    if (!data) {pciDeviceFree(hid, mem); return NULL;}

    UMatData* u = new UMatData(this);
    if (!u) {
      CV_Error(CV_HalMemErr, "pci UMatData failed");
      return NULL;
    }

    u->data = u->origdata = static_cast<uchar*>(data);
    u->size = total;
    u->addr = addr;
    u->hid = hid;
    u->mem = mem;

    if (data0) {
      u->flags |= UMatData::USER_ALLOCATED;
    }
    return u;
  }

  UMatData* allocate(int total, int id) const
  {
    bm_handle_t hid = bmcv::getCard(id);

    bm_uint64 addr;
    bm_device_mem_t mem;
    if (!pciDeviceMalloc(total, hid, id, &mem, &addr)) return NULL;
    void* data = pciSystemMalloc(total);
    if (!data) {pciDeviceFree(hid, mem); return NULL;}


    UMatData* u = new UMatData(this);
    if (!u) {
      CV_Error(CV_HalMemErr, "pci UMatData failed");
      return NULL;
    }

    u->data = u->origdata = static_cast<uchar*>(data);
    u->size = total;
    u->addr = addr;
    u->hid = hid;
    u->mem = mem;
    return u;
  }

  UMatData* allocate(int total, void *data0, bm_uint64 addr0, int fd0, int id) const
  {
    bm_handle_t hid = bmcv::getCard(id);

    bm_uint64 addr = (fd0>=0) ? addr0 : 0;
    bm_device_mem_t mem;
    if(fd0 >= 0) mem = bm_mem_from_device(addr0, total);
    else { if (!pciDeviceMalloc(total, hid, id, &mem, &addr)) return NULL; }

    void* data = data0 ? data0 : pciSystemMalloc(total);
    if (!data) return NULL;

    UMatData* u = new UMatData(this);
    if (!u) {
      CV_Error(CV_HalMemErr, "pci UMatData failed");
      return NULL;
    }

    u->data = u->origdata = static_cast<uchar*>(data);
    u->size = total;
    u->addr = addr;
    u->hid = hid;
    u->mem = mem;

    if (data0) {
        u->flags |= UMatData::USER_ALLOCATED;
    }
    if (fd0 >= 0){
        u->flags |= UMatData::DEVICE_MEM_ATTACHED;
    }
    return u;
  }

  bool allocate(UMatData* u, AccessFlag /*accessFlags*/, UMatUsageFlags /*usageFlags*/) const
  {
    if(!u) return false;
    return true;
  }

  void deallocate(UMatData* u) const
  {
    if (!u) return;

    CV_Assert(u->urefcount == 0);
    CV_Assert(u->refcount == 0);
    if (!(u->flags & UMatData::DEVICE_MEM_ATTACHED))
        pciDeviceFree(u->hid, u->mem);
    if (!(u->flags & UMatData::USER_ALLOCATED))
    {
        pciSystemFree(u->origdata);
        u->origdata = 0;
    }
    delete u;
  }

private:
  bool pciDeviceMalloc(size_t size, bm_handle_t hid, int id, bm_device_mem_t* mem, bm_uint64* addr) const
  {
    int heap_id = BM_CARD_HEAP(id);
    if (heap_id == 0) {
        heap_id = (1 << HEAP_VPP) | (1 << HEAP_VPU);
    }
    bm_status_t ret = bm_malloc_device_byte_heap_mask(hid, mem, heap_id, size);
    if (ret != BM_SUCCESS) {
        CV_Error(CV_HalMemErr, "pci bm_malloc_device_byte_heap_mask failed");
        return false;
    }

    *addr = bm_mem_get_device_addr(*mem);
    return true;
  }

  void* pciSystemMalloc(size_t size) const
  {
    void *p = fastMalloc(size);
    if (!p) {
      CV_Error(CV_HalMemErr, "pci fastMalloc failed");
      return NULL;
    }

    return p;
  }

  void pciDeviceFree(bm_handle_t hid, bm_device_mem_t mem) const
  {
    if(mem.size) bm_free_device(hid, mem);
  }

  void pciSystemFree(void *vaddr) const
  {
    if (vaddr) fastFree(vaddr);
  }
};

MatAllocator* getPCIAllocator()
{
  CV_SINGLETON_LAZY_INIT(MatAllocator, new PCIAllocator())
}

#endif

namespace
{
  MatAllocator* volatile g_matAllocator = NULL;
}

#ifndef WIN32
MatAllocator* getIONAllocator()
{
  CV_SINGLETON_LAZY_INIT(MatAllocator, new IONAllocator())
}
#endif

MatAllocator* getAllocator()
{
  if (g_matAllocator == NULL)
  {
    cv::AutoLock lock(cv::getInitializationMutex());
    if (g_matAllocator == NULL)
    {
#ifdef USING_SOC
      g_matAllocator = getIONAllocator();
#else
      g_matAllocator = getPCIAllocator();
#endif
    }
  }
  return g_matAllocator;
}

}}
