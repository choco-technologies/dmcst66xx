#ifndef DMCST66XX_H
#define DMCST66XX_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "dmod_types.h"
#include "dmcst66xx_defs.h"
#include "dmcst66xx_types.h"

/**
 * @brief Check a configuration without touching hardware
 *
 * @return true when the screen size, the address and the number of contacts make sense.
 */
dmod_dmcst66xx_api(1.0, bool, _validate_config, ( const dmcst66xx_config_t *config ));

/** Bytes read from the report register: the header of 4 bytes and 5 bytes for each of the (at most 5) entries */
#define DMCST66XX_REPORT_SIZE       (4U + 5U * DMCST66XX_MAX_CONTACTS)

/** The first read of a report: the header and one entry */
#define DMCST66XX_REPORT_HEAD_SIZE  9U

/** The chip has no report yet (it reports only what changed): the report registers read as zeros */
#define DMCST66XX_REPORT_TYPE_NONE  0x00U
/** The report holds positions (0xF0 is a gesture and 0xE0 the proximity: not used here) */
#define DMCST66XX_REPORT_TYPE_POS   0xFFU

/**
 * @brief Decode the report of the chip
 *
 * @param report        DMCST66XX_REPORT_SIZE bytes read from the register 0xD0070000: the 16-bit checksum (little-endian), the type of
 *                      the report, the number of fingers (low nibble) and of keys (high nibble) in the byte 3, then 5 bytes for each
 *                      entry, the keys first
 * @param size          Bytes in report
 * @param contacts      Output: the fingers (the keys are skipped)
 * @param max_contacts  Size of contacts
 *
 * @return Fingers decoded, 0 when the report holds no position (the chip had nothing new); -EBADMSG when it is not a valid report
 *         (too short, more entries than the chip has, or a wrong checksum: the sum of the entries and 0x55)
 */
dmod_dmcst66xx_api(1.0, int, _parse_report, ( const uint8_t *report, size_t size, dmcst66xx_raw_contact_t *contacts, uint8_t max_contacts ));

/**
 * @brief Map a raw point to screen coordinates (scaling, swap, mirroring), the result is clipped to the screen
 *
 * @param config  The configuration; raw_width and raw_height must be set (not 0)
 */
dmod_dmcst66xx_api(1.0, void, _map_point, ( const dmcst66xx_config_t *config, uint16_t raw_x, uint16_t raw_y, uint16_t *x, uint16_t *y ));

#endif // DMCST66XX_H
