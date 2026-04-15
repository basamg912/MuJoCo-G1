#ifndef __TIMER_DEF__
#define __TIMER_DEF__

#include <chrono>

#define TIMER_START(label) \
    auto __start_##label = std::chrono::high_resolution_clock::now()

#define TIMER_END(label) \
    { \
        auto __end_##label = std::chrono::high_resolution_clock::now(); \
        std::chrono::duration<double, std::milli> __elapsed_##label = __end_##label - __start_##label; \
        std::cout << "[TIMER] " #label " took " << __elapsed_##label.count() << " ms" << std::endl; \
    }


#endif