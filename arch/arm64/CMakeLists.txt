# SPDX-License-Identifier: Apache-2.0

set_property(GLOBAL PROPERTY PROPERTY_OUTPUT_FORMAT elf64-littleaarch64)

add_subdirectory(core)
add_subdirectory_ifdef(CONFIG_SOC_XENVM xen)