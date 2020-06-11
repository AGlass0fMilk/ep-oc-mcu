/**
 * ep-oc-mcu
 * Embedded Planet Open Core for Microcontrollers
 *
 * Built with ARM Mbed-OS
 *
 * Copyright (c) 2019-2020 Embedded Planet, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "platform/mbed_retarget.h"
#include "drivers/SerialWireOutput.h"

#if MBED_CONF_EP_UTILS_SWO_DEBUG_OUTPUT

/**
 * Retarget the Mbed stdio stream to SWO
 */
mbed::FileHandle *mbed::mbed_target_override_console(int fd)
{
    static mbed::SerialWireOutput swo;
    return &swo;
}

#endif


