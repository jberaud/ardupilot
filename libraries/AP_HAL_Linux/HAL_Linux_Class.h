#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_Linux_Namespace.h"
#include "UltraSound_Bebop.h"

class HAL_Linux : public AP_HAL::HAL {
public:
    UltraSound_Bebop *ultraSound;
    HAL_Linux();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
};
