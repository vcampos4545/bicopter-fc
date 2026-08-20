#include "task_util.h"

bool task_should_decimate(uint32_t cycle_count, uint32_t decimation)
{
    if (decimation <= 1) {
        return true;
    }
    return (cycle_count % decimation) == 0;
}
