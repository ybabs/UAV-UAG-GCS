#include "gcs/utils/utils.h"

static bool is_random_assigned = false;

std::default_random_engine& randomEngine()
{
    static std::default_random_engine e;
    if(!is_random_assigned)
    {
        std::random_device r;
        e = std::default_random_engine(r());
        is_random_assigned = true;
    }
    return e;
}
