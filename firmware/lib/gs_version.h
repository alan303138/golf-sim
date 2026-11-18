/* Firmware version. Keep in sync with project(VERSION) in firmware/CMakeLists.txt.
 * Reported in the INFO message; the OTA agent verifies it after flashing. */
#ifndef GS_VERSION_H
#define GS_VERSION_H

#define GS_FW_VERSION_MAJOR 1
#define GS_FW_VERSION_MINOR 0
#define GS_FW_VERSION_PATCH 0
#define GS_FW_VERSION_STR   "1.0.0"

#ifndef GS_HW_REVISION
#define GS_HW_REVISION 1
#endif

/* Wire protocol version (see PROTOCOL.md). A different major is incompatible. */
#define GS_PROTO_VERSION_MAJOR 1
#define GS_PROTO_VERSION_MINOR 0

#endif
