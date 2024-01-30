#include "precomp.hpp"
#include "opencv2/core/cv_bmcpu.hpp"
#include <thread>

#if defined(linux) || defined(__linux) || defined(ANDROID)
#include <sys/stat.h>
#include <unistd.h>
#include <dlfcn.h>
#include <link.h>
#include <unistd.h>
#elif defined WIN32
#include <io.h>
#endif

#define LIBRARY_NUM  17
#define MAX_CHIP_NUM 256
#define BMCPU_LOG_LEVEL 1 // INFO
#define BMCPU_FOLDER_NAME  "bmcpu/"
#define FIP_FILENAME    "fip.bin"
#define RAMBOOT_ITB_FILENAME    "ramboot_rootfs.itb"
#define BMCPU_LOG_FILE    "./bmcpu_ocv.log"
#define BMCPU_PROFILING     0

using namespace std;
namespace cv{

#if !defined(USING_SOC) && defined(BM1684_CHIP) && defined(ENABLE_BMCPU)

class InternalBMCpuRegister
{
public:
    InternalBMCpuRegister()
    {
        AutoLock lock(getInitializationMutex());
        if (!bmcpu_dev_initialized)
        {
            int i;
            thread threads[MAX_CHIP_NUM];
            if (BM_SUCCESS != bm_dev_getcount(&dev_num)){
                BMCPU_LOG(stdout, "no sophon device found!\n");
                return;
            }
            if (dev_num > MAX_CHIP_NUM){
                BMCPU_LOG(stdout, "too many device number %d\n", dev_num);
                return;
            }

            BMCPU_LOG(stdout, "total %d devices need to enable on-chip CPU. It may need serveral minutes \
                    for loading, please be patient....\n", dev_num);
#if BMCPU_PROFILING
            int64_t start = getTickCount();
#endif
#if 0
            /* get filename */
            if (0 > getLocationPath(library_path, sizeof(library_path))) {
                BMCPU_LOG(stdout, "can not find bmcpu path\n");
                return;
            }

            if (strlen(library_path) + 1 + strlen(FIP_FILENAME) <= sizeof(fip_path)){
                strncpy(fip_path, library_path, strlen(library_path)+1);
                strcat(fip_path, FIP_FILENAME);
            } else {
                BMCPU_LOG(stdout, "fip file name is too long\n");
                return;
            }
#ifdef WIN32
            if (_access(fip_path, 0) != 0){
#else
            if (access(fip_path, F_OK) != 0){
#endif
                BMCPU_LOG(stdout, "fip_paht(%s) is not exit!\n", fip_path);
                return;
            }

            if (strlen(library_path) + 1 + strlen(RAMBOOT_ITB_FILENAME) <= sizeof(itb_path)){
                strncpy(itb_path, library_path, strlen(library_path)+1);
                strcat(itb_path, RAMBOOT_ITB_FILENAME);
            } else {
                BMCPU_LOG(stdout, "itb file name is too long\n");
                return;
            }
#ifdef WIN32
            if (_access(itb_path, 0) != 0){
#else
            if (access(itb_path, F_OK) != 0){
#endif
                BMCPU_LOG(stdout, "itb_paht(%s) is not exit!\n", itb_path);
                return;
            }
#endif

            for (i = 0; i < dev_num; i++)
                threads[i] = thread(load_fn, i);
            for (i = 0; i < dev_num; i++){
                threads[i].join();
                BMCPU_LOG(stdout, "%d/%d devices finished\n", i, dev_num);
            }
#if BMCPU_PROFILING
            BMCPU_LOG(stdout, "[Profiling]device enabling uses %ld tickers\n", getTickCount() - start);
#endif
            for (i = 0; i < dev_num; i++){
                if (device_handle[i] && process_handle[i])
                    BMCPU_LOG(stdout, "device %d on-chip CPU enabled!\n", i);
                else
                    BMCPU_LOG(stdout, "device %d failed to enable on-chip CPU! If not using bmcpu_opencv function, please ignore it.\n", i);
            }
            bmcpu_dev_initialized = true;
        }
        return;
    }

    ~InternalBMCpuRegister()
    {
        BMCPU_LOG(stdout, "deconstructor function is called\n");
        if (bmcpu_dev_initialized){
            int i;

            for (i = 0; i < dev_num; i++){
                if (process_handle[i]){
                    BMCPU_LOG(stdout, "bmcpu close process %d is called\n", i);
                    bmcpu_close_process(device_handle[i], process_handle[i], -1);
                }
                if (device_handle[i])
                    bm_dev_free(device_handle[i]);
                process_handle[i] = 0;
                device_handle[i] = NULL;
            }
        }
        bmcpu_dev_initialized = false;
    }

    static bm_handle_t device_handle[MAX_CHIP_NUM];
    static int process_handle[MAX_CHIP_NUM];
    int dev_num = 0;

private:
    static char library_path[256];
    static char fip_path[256];
    static char itb_path[256];

    static string library_file[LIBRARY_NUM];
    static bool bmcpu_dev_initialized;

    static void load_fn(int id){
        bm_status_t ret;
        bm_cpu_status_t ret_bmcpu;

        device_handle[id] = NULL;
        process_handle[id] = 0;

        ret = bm_dev_request(&device_handle[id], id);
        if ((ret != BM_SUCCESS) || (device_handle[id] == NULL)) {
            BMCPU_LOG(stdout, "device %d bm_dev_request error, ret = %d\n", id, ret);
            goto error_exit;
        }

        ret_bmcpu =  bmcpu_get_cpu_status(device_handle[id]);
        if (ret_bmcpu != BMCPU_RUNNING) {
            // BMCPU_LOG(stdout, "Warning: device %d start cpu error! If bmcpu_opencv function is not used, please ignore this warning\n", id);
            goto error_exit;
        }

        // ret = bmcpu_start_cpu(device_handle[id], fip_path,  itb_path);
        // if (ret != BM_SUCCESS) {
        //     BMCPU_LOG(stdout, "Warning: device %d start cpu error! If bmcpu_opencv function is not used, please ignore this warning\n", id);
        //     goto error_exit;
        // }

        ret = bmcpu_set_log(device_handle[id], BMCPU_LOG_LEVEL, 0, -1);
        if (ret != 0) {
            BMCPU_LOG(stdout, "ERROR!!! device %d set_log function error!\n", id);
            goto error_exit;
        }

        process_handle[id] = bmcpu_open_process(device_handle[id], 0, -1);
        if (process_handle[id] < 0) {
            BMCPU_LOG(stdout, "ERROR!!! device %d open process ret: %d\n", id, process_handle[id]);
            process_handle[id] = 0;
            goto error_exit;
        }
#if 0
        // ramboot.itb will load those library, so remove load library here
        for (int i = 0; i < LIBRARY_NUM; i++){
            string lib_full_name = library_path;
            lib_full_name += library_file[i];

            if (strlen(lib_full_name.c_str()) > 0){
                ret = bmcpu_load_library(device_handle[id], process_handle[id], (char *)lib_full_name.c_str(), -1);
                if (ret != BM_SUCCESS) {
                    bmcpu_get_log(device_handle[id], process_handle[id], (char *)BMCPU_LOG_FILE, -1);
                    BMCPU_LOG(stdout, "ERROR!!! device %d load library %s ret: %d\n", id, lib_full_name.c_str(), ret);
                    goto error_exit;
                }
            } else
                break;
        }
#endif
        return;

error_exit:
        if (process_handle[id]){BMCPU_LOG(stdout, "close process is called\n"); bmcpu_close_process(device_handle[id], process_handle[id], -1); process_handle[id] = 0;};
        if (device_handle[id]) {bm_dev_free(device_handle[id]); device_handle[id] = NULL;};
        return;
    };

   int getLocationPath(char *path, int size)
    {
#if defined(linux) || defined(__linux) || defined(ANDROID)
        struct link_map *map;
        int ret = -1;

        void *handle = dlopen("libopencv_core.so", RTLD_NOW);
        if (handle == NULL) {
            BMCPU_LOG(stdout, "dlopen() failed: %s\n", dlerror());
            return -1;
        }
        if(dlinfo(handle, RTLD_DI_LINKMAP, &map) != -1) {
            char *ptr = strrchr(map->l_name, '/');

            if (ptr){
                int name_len = ptr - map->l_name + 1;

                if(name_len > 0 && (name_len + strlen(BMCPU_FOLDER_NAME) + 1) <= (size_t)size) {
                    memset(path, 0, size);
                    strncpy(path, map->l_name, name_len);
                    strcat(path, BMCPU_FOLDER_NAME);
                    BMCPU_LOG(stdout, "bmcpu full path: %s\n", path);
                    ret = 0;
                }
            }
        }

        if (dlclose(handle) != 0)
            BMCPU_LOG(stdout, "WARNING! dlcose failed\n");

        return ret;
#else
        BMCPU_LOG(stdout, "Only linux platform is supported now to detect library path\n");
        return 0;
 #endif
    }
};


string InternalBMCpuRegister::library_file[LIBRARY_NUM] =
{"libbmvppapi.so",
 "libbmion.so",
 "libyuv.so",
 "libbmlib.so",
 "libbmvideo.so",
 "libbmjpulite.so",
 "libbmjpuapi.so",
 "libbmvpulite.so",
 "libbmvpuapi.so",
 "libbmcv.so",
 "libavutil.so.56",
 "libswresample.so.3",
 "libswscale.so.5",
 "libavcodec.so.58",
 "libavformat.so.58",
 "libopencv_world.so.4.1",
 "libcvwrapper.so"};

bool InternalBMCpuRegister::bmcpu_dev_initialized = false;
bm_handle_t InternalBMCpuRegister::device_handle[MAX_CHIP_NUM];
int InternalBMCpuRegister::process_handle[MAX_CHIP_NUM];
char InternalBMCpuRegister::library_path[256];
char InternalBMCpuRegister::fip_path[256];
char InternalBMCpuRegister::itb_path[256];

static InternalBMCpuRegister _bmcpu_dev;
#endif

/*
 * start to define bmcpu_sender class
 */
BMCpuSender::BMCpuSender(int dev_id, int size)
    :_buffer(NULL), _param_byte_size(0), _buffer_size(0),
     _device_id(-1), _map_vaddr(NULL), _handle(NULL), _process_handle(0)
{
    bm_status_t ret;

    _dev_mem.size = 0;

#if !defined(USING_SOC) && defined(BM1684_CHIP) && defined(ENABLE_BMCPU)
    if (dev_id >= _bmcpu_dev.dev_num){
        BMCPU_LOG(stdout, "BMCpuSender device %d exceed maximum device number %d\n", dev_id, _bmcpu_dev.dev_num);
        return;
    }
    if (size <= 0){
        BMCPU_LOG(stdout, "BMCpuSender illegal size %d\n", size);
        return;
    }
    if (!_bmcpu_dev.device_handle[dev_id] || !_bmcpu_dev.process_handle[dev_id]){
        BMCPU_LOG(stdout, "BMCpuSender device %d is not initialized!\n", dev_id);
        return;
    }

    _handle = _bmcpu_dev.device_handle[dev_id];
    _process_handle = _bmcpu_dev.process_handle[dev_id];

#if BMCPU_PROFILING
    int64_t start = getTickCount();
#endif
    ret = bm_malloc_device_byte(_handle, &_dev_mem, size);
    if (ret != BM_SUCCESS) {
        BMCPU_LOG(stdout, "BMCpuSender device %d malloc device mem error!\n", dev_id);
        return;
    }

    _map_vaddr = (unsigned char *)bmcpu_map_phys_addr(_handle, _process_handle, (void *)(_dev_mem.u.device.device_addr), size, -1);
    if (_map_vaddr == NULL) {
        BMCPU_LOG(stdout, "BMCpuSender device %d map phys addr error!\n", dev_id);
        return;
    }

    _buffer = (unsigned char *)malloc(size);
    if (_buffer == NULL) {
        BMCPU_LOG(stdout, "BMCpuSender device %d malloc buffer error!\n", dev_id);
        return;
    }
#if BMCPU_PROFILING
    BMCPU_LOG(stdout, "[Profiling]BMCpuSender initialization uses %ld tickers\n", getTickCount() - start);
#endif

    _device_id = dev_id;
    _buffer_size = size;
    _param_byte_size = sizeof(int64_t);  // for memory alignement
#endif

    return;
}

BMCpuSender::~BMCpuSender()
{
#if !defined(USING_SOC) && defined(BM1684_CHIP) && defined(ENABLE_BMCPU)
    if (_map_vaddr)
        bmcpu_unmap_phys_addr(_handle, _process_handle, \
                (void *)(_dev_mem.u.device.device_addr), -1);
    if (_dev_mem.size)
        bm_free_device(_handle, _dev_mem);
    if (_buffer)
        free(_buffer);

    _map_vaddr = NULL;
    _buffer = NULL;
    _dev_mem.size = 0;
#endif

    return;
}

int BMCpuSender::run(const String& function_name)
{
    int ret = 0;

    if (!_buffer_size){
        BMCPU_LOG(stdout, "BMCpuSender is not properly initialized\n");
        return -1;
    }

#if !defined(USING_SOC) && defined(BM1684_CHIP) && defined(ENABLE_BMCPU)

#if BMCPU_PROFILING
    int64_t start = getTickCount();
#endif
    /* upload parameters */
    *(int32_t *)_buffer = _param_byte_size;
    *(int32_t *)(_buffer+sizeof(int)) = _buffer_size;
    ret = bm_memcpy_s2d(_handle, _dev_mem, _buffer);
    if (ret != 0) {
        BMCPU_LOG(stdout, "BMCpuSender device %d copy data to device mem error!\n", _device_id);
        return -1;
    }
#if BMCPU_PROFILING
    BMCPU_LOG(stdout, "[Profiling]BMCpuSender run s2d uses %ld tickers\n", getTickCount() - start);
    start = getTickCount();
#endif

    /* execute function */
    ret = bmcpu_exec_function(_handle,
                              _process_handle,
                              (char *)function_name.c_str(),
                              (void *)&_map_vaddr,
                              sizeof(void *),
                              -1);
    if (ret != 0) {
        BMCPU_LOG(stdout, "BMCpuSender device %d exec function error!\n", _device_id);
        bmcpu_get_log(_handle, _process_handle, (char *)BMCPU_LOG_FILE, -1);
        return -1;
    }

#if BMCPU_PROFILING
    BMCPU_LOG(stdout, "[Profiling]BMCpuSender run uses %ld tickers\n", getTickCount() - start);
    start = getTickCount();
#endif

    /* download parameter */
    ret = bm_memcpy_d2s(_handle, _buffer, _dev_mem);
    if (ret != 0) {
        BMCPU_LOG(stdout, "BMCpuSender device %d copy data to device mem error!\n", _device_id);
        return -1;
    }
    _param_byte_size = sizeof(int64_t);

#if BMCPU_PROFILING
    BMCPU_LOG(stdout, "[Profiling]BMCpuSender run d2s uses %ld tickers\n", getTickCount() - start);
#endif
#endif
    return ret;
}

int BMCpuSender::size(Mat &m)
{
    int ret = 0;

    ret += sizeof(Mat);
    if (m.empty())
        return ret;

    if (m.dims > 2)
        ret += m.dims*sizeof(m.step.p[0]) + (m.dims+1)*sizeof(m.size.p[0]);
    if (m.u){
        ret += sizeof(UMatData);
        if (m.avOK())
            ret += sizeof(AVFrame);
    }

    return ret;
}

int BMCpuSender::put(Mat &m)
{
    int ret = 0;
    if (!_buffer){
        BMCPU_LOG(stdout, "BMCpuSender device %d buffer is not allocated\n", _device_id);
        return -1;
    }

    if (!m.empty() && !(m.u && m.u->addr)){
        BMCPU_LOG(stdout, "BMCpuSender Mat device buffer is not allocated\n");
        return -2;
    }
    if (_param_byte_size + size(m) > (size_t)_buffer_size){
        BMCPU_LOG(stdout, "BMCpuSender no sufficient memory for parameters _param_byte_size=%d m_len=%d _buffer_size=%d\n", _param_byte_size, size(m), _buffer_size);
        return -1;
    }

    memcpy(_buffer+_param_byte_size, &m, sizeof(Mat));
    _param_byte_size += sizeof(Mat);
    if (m.dims > 2){
        int step_buf_size = m.dims*sizeof(m.step.p[0]) + (m.dims+1)*sizeof(m.size.p[0]);
        memcpy(_buffer+_param_byte_size, m.step.p, step_buf_size);
        _param_byte_size += step_buf_size;
    }

    if (m.u){
        memcpy(_buffer+_param_byte_size, m.u, sizeof(UMatData));
        _param_byte_size += sizeof(UMatData);
        if (m.avOK()){
            memcpy(_buffer+_param_byte_size, m.u->frame, sizeof(AVFrame));
            _param_byte_size += sizeof(AVFrame);
        }
    }
    _param_byte_size = OCV_BMCPU_ALIGN(_param_byte_size, sizeof(int64_t));

    return ret;
}

int BMCpuSender::skip(Mat &m)
{
    int ret = 0;
    if (!_buffer){
        BMCPU_LOG(stdout, "BMCpuSender device %d buffer is not allocated\n", _device_id);
        return -1;
    }
    if (_param_byte_size + size(m) > (size_t)_buffer_size){
        BMCPU_LOG(stdout, "BMCpuSender no sufficient memory for parameters\n");
        return -1;
    }

    _param_byte_size += size(m);
    _param_byte_size = OCV_BMCPU_ALIGN(_param_byte_size, sizeof(int64_t));

    return ret;
}

int BMCpuSender::get(Mat &m)
{
    int ret = 0;
    int offset = _param_byte_size;
    if (!_buffer){
        BMCPU_LOG(stdout, "BMCpuSender device %d buffer is not allocated\n", _device_id);
        return -1;
    }

    ret = skip(m);
    if (ret == 0){
        memcpy(&m, _buffer+offset, sizeof(Mat));
        offset += sizeof(Mat);
        if (m.dims > 2){
            int step_buf_size = m.dims*sizeof(m.step.p[0]) + (m.dims+1)*sizeof(m.size.p[0]);
            memcpy(m.step.p, _buffer+offset, step_buf_size);
            offset += step_buf_size;
        }
        if (m.u){
            memcpy(m.u, _buffer+offset, sizeof(UMatData));
            offset += sizeof(UMatData);
            if (m.avOK()){
                memcpy(m.u->frame, _buffer+offset, sizeof(AVFrame));
            }
        }
    }

    return ret;
}

int BMCpuSender::put(unsigned char &value)
{
    return put_basic_data(value);
}

int BMCpuSender::skip(unsigned char &value)
{
    return skip_basic_data(value);
}

int BMCpuSender::get(unsigned char &value)
{
    return get_basic_data(value);
}

int BMCpuSender::put(int &value)
{
    return put_basic_data(value);
}

int BMCpuSender::skip(int &value)
{
    return skip_basic_data(value);
}

int BMCpuSender::get(int &value)
{
    return get_basic_data(value);
}

int BMCpuSender::put(float &value)
{
    return put_basic_data(value);
}

int BMCpuSender::skip(float &value)
{
    return skip_basic_data(value);
}

int BMCpuSender::get(float &value)
{
    return get_basic_data(value);
}

int BMCpuSender::put(double &value)
{
    return put_basic_data(value);
}

int BMCpuSender::skip(double &value)
{
    return skip_basic_data(value);
}

int BMCpuSender::get(double &value)
{
    return get_basic_data(value);
}

int BMCpuSender::put(bool &value)
{
    return put_basic_data(value);
}

int BMCpuSender::skip(bool &value)
{
    return skip_basic_data(value);
}

int BMCpuSender::get(bool &value)
{
    return get_basic_data(value);
}

int BMCpuSender::put(void *m, int byte_size)
{
    int ret = 0;
    if (!_buffer){
        BMCPU_LOG(stdout, "BMCpuSender device %d buffer is not allocated\n", _device_id);
        return -1;
    }
    if (_param_byte_size + sizeof(int) + byte_size > (size_t)_buffer_size){
        BMCPU_LOG(stdout, "BMCpuSender no sufficient memory for parameters\n");
        return -1;
    }

    if (m == NULL) byte_size = 0;
    *(int *)(_buffer+_param_byte_size) = byte_size;
    _param_byte_size += sizeof(int);
    if (byte_size > 0){
        memcpy(_buffer+_param_byte_size, m, byte_size);
        _param_byte_size += byte_size;
    }
    _param_byte_size = OCV_BMCPU_ALIGN(_param_byte_size, sizeof(int64_t));

    return ret;

}

int BMCpuSender::skip(void *m, int &byte_size)
{
    int ret = 0;

    if (!_buffer){
        BMCPU_LOG(stdout, "BMCpuSender device %d buffer is not allocated\n", _device_id);
        return -1;
    }
    if (_param_byte_size + sizeof(int) > (size_t)_buffer_size){
        BMCPU_LOG(stdout, "BMCpuSender no sufficient memory for parameters\n");
        return -1;
    }

    byte_size = *(int *)(_buffer+_param_byte_size);

    if (_param_byte_size + sizeof(int) + byte_size > (size_t)_buffer_size){
        BMCPU_LOG(stdout, "BMCpuSender no sufficient memory for parameters\n");
        return -1;
    }
    _param_byte_size += sizeof(int);
    _param_byte_size += byte_size;
    _param_byte_size = OCV_BMCPU_ALIGN(_param_byte_size, sizeof(int64_t));

    return ret;
}

int BMCpuSender::get(void **m, int &byte_size)
{
    int ret = 0;
    int offset = _param_byte_size;

    if (!_buffer){
        BMCPU_LOG(stdout, "BMCpuSender device %d buffer is not allocated\n", _device_id);
        return -1;
    }

    ret = skip(*m, byte_size);
    if (ret == 0){
        byte_size = *(int *)(_buffer+offset);
        offset += sizeof(int);
        if (byte_size > 0)
            *m = (void *)(_buffer+offset);
        else
            *m = NULL;
    }

    return ret;
}

int BMCpuSender::put(String &text)
{
    int ret = 0;
    if (!_buffer){
        BMCPU_LOG(stdout, "BMCpuSender device %d buffer is not allocated\n", _device_id);
        return -1;
    }
    if (_param_byte_size + sizeof(int) + text.size() + 1 > (size_t)_buffer_size){
        BMCPU_LOG(stdout, "BMCpuSender no sufficient memory for parameters\n");
        return -1;
    }

    *(int *)(_buffer+_param_byte_size) = text.size() + 1;
    _param_byte_size += sizeof(int);
    memcpy(_buffer+_param_byte_size, text.c_str(), text.size()+1);
    _param_byte_size += text.size()+1;
    _param_byte_size = OCV_BMCPU_ALIGN(_param_byte_size, sizeof(int64_t));

    return ret;
}

int BMCpuSender::skip(String &text)
{
    int ret = 0;
    int text_size;

    if (!_buffer){
        BMCPU_LOG(stdout, "BMCpuSender device %d buffer is not allocated\n", _device_id);
        return -1;
    }
    if (_param_byte_size + sizeof(int) > (size_t)_buffer_size){
        BMCPU_LOG(stdout, "BMCpuSender no sufficient memory for parameters\n");
        return -1;
    }

    text_size = *(int *)(_buffer+_param_byte_size);

    if (_param_byte_size + sizeof(int) + text_size > (size_t)_buffer_size){
        BMCPU_LOG(stdout, "BMCpuSender no sufficient memory for parameters\n");
        return -1;
    }
    _param_byte_size += sizeof(int);
    _param_byte_size += text_size;
    _param_byte_size = OCV_BMCPU_ALIGN(_param_byte_size, sizeof(int64_t));

    return ret;
}

int BMCpuSender::get(String &text)
{
    int ret = 0;
    int text_size = 0;
    const char *buf;
    int offset = _param_byte_size;

    if (!_buffer){
        BMCPU_LOG(stdout, "BMCpuSender device %d buffer is not allocated\n", _device_id);
        return -1;
    }

    ret = skip(text);
    if (ret == 0){
        text_size = *(int *)(_buffer+offset);
        offset += sizeof(int);
        buf = (const char *)(_buffer+offset);
        text.assign(buf, text_size-1);
    }

    return ret;
}

}
