/**
 * @file firmware.h
 * @brief Defines firmware-wide constants like name and version.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#pragma once

#include <string>

static constexpr auto *FIRMWARE_NAME = "Flight32";
static constexpr auto *FIRMWARE_VERSION = "0.3.0";

static constexpr auto get_firmware_version() { return FIRMWARE_VERSION; };
