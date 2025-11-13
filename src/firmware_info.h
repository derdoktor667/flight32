/**
 * @file firmware_info.h
 * @brief Defines the FirmwareInfo class for managing firmware-wide constants like name and version.
 * @author Wastl Kraus
 * @date 2025-11-13
 * @license MIT
 */

#pragma once

#include <string>
#include "config.h"

class FirmwareInfo
{
public:
    static constexpr const char *getFirmwareVersion() { return FIRMWARE_VERSION; }
};
