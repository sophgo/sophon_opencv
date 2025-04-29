#ifndef __BM_CORE_HAL_HPP
#define __BM_CORE_HAL_HPP

#ifndef WIN32
#include "opencv2/core/ion.hpp"
#endif

namespace cv { namespace hal {

CV_EXPORTS MatAllocator* getAllocator();
CV_EXPORTS MatAllocator* getPCIAllocator();
CV_EXPORTS MatAllocator* getIONAllocator();
}}

#endif
