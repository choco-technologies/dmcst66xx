#define DMOD_ENABLE_REGISTRATION ON
#include "dmod_test.h"
#include "dmcst66xx.h"
#include <errno.h>
#include <string.h>

static dmcst66xx_config_t screen(void)
{
    dmcst66xx_config_t c = { .width = 240, .height = 320, .raw_width = 240, .raw_height = 320, .address = 0x1A,
                            .max_contacts = 5, .poll_ms = 10, .release_ms = 50 };
    return c;
}

DMOD_TEST_STEP(validate_accepts_the_board)
{
    dmcst66xx_config_t c = screen();
    DMOD_TEST_EXPECT_TRUE(dmcst66xx_validate_config(&c));
    c.raw_width = 0;                        /* the range of the chip */
    c.raw_height = 0;
    DMOD_TEST_EXPECT_TRUE(dmcst66xx_validate_config(&c));
}

DMOD_TEST_STEP(validate_rejects_bad_configurations)
{
    dmcst66xx_config_t c = screen();
    DMOD_TEST_EXPECT_FALSE(dmcst66xx_validate_config(NULL));
    c.width = 0;
    DMOD_TEST_EXPECT_FALSE(dmcst66xx_validate_config(&c));
    c = screen();
    c.address = 0x80;
    DMOD_TEST_EXPECT_FALSE(dmcst66xx_validate_config(&c));
    c = screen();
    c.max_contacts = 6;
    DMOD_TEST_EXPECT_FALSE(dmcst66xx_validate_config(&c));
    c.max_contacts = 0;
    DMOD_TEST_EXPECT_FALSE(dmcst66xx_validate_config(&c));
    c = screen();
    c.raw_width = 5000;
    DMOD_TEST_EXPECT_FALSE(dmcst66xx_validate_config(&c));
}

/* A report: the entries are the 5-byte records; the checksum, the type 0xFF and the counts are filled in */
static void make_report(uint8_t *report, uint8_t fingers, uint8_t keys, const uint8_t *entries)
{
    memset(report, 0, DMCST66XX_REPORT_SIZE);
    uint8_t n = (uint8_t)(fingers + keys);
    memcpy(report + 4, entries, 5U * n);
    uint16_t sum = 0x55;
    for (unsigned i = 0; i < 5U * n; i++)
        sum = (uint16_t)(sum + report[4 + i]);
    report[0] = (uint8_t)sum;
    report[1] = (uint8_t)(sum >> 8);
    report[2] = 0xFF;
    report[3] = (uint8_t)(fingers | (keys << 4));
}

DMOD_TEST_STEP(parse_report_without_news)
{
    uint8_t report[DMCST66XX_REPORT_SIZE] = { 0 };          /* what an idle chip reads: zeros, no checksum to check */
    dmcst66xx_raw_contact_t c[5];
    DMOD_TEST_EXPECT_EQ(dmcst66xx_parse_report(report, sizeof(report), c, 5), 0);
    report[2] = 0xF0;                                       /* a gesture: not a position */
    DMOD_TEST_EXPECT_EQ(dmcst66xx_parse_report(report, sizeof(report), c, 5), 0);
    DMOD_TEST_EXPECT_EQ(dmcst66xx_parse_report(report, 3, c, 5), -EBADMSG);
    DMOD_TEST_EXPECT_EQ(dmcst66xx_parse_report(NULL, sizeof(report), c, 5), -EBADMSG);
}

DMOD_TEST_STEP(parse_report_one_finger)
{
    /* x = 0x0AB, y = 0x123: x low 0xAB, y low 0x23, pressure 0x40, high nibbles y 1 / x 0, finger 3 touching (event 1) */
    const uint8_t entry[5] = { 0xAB, 0x23, 0x40, 0x10, 0x13 };
    uint8_t report[DMCST66XX_REPORT_SIZE];
    dmcst66xx_raw_contact_t c[5];
    make_report(report, 1, 0, entry);
    DMOD_TEST_EXPECT_EQ(dmcst66xx_parse_report(report, sizeof(report), c, 5), 1);
    DMOD_TEST_EXPECT_EQ(c[0].x, 0x0AB);
    DMOD_TEST_EXPECT_EQ(c[0].y, 0x123);
    DMOD_TEST_EXPECT_EQ(c[0].id, 3);
    DMOD_TEST_EXPECT_EQ(c[0].pressure, 0x40);
    DMOD_TEST_EXPECT_TRUE(c[0].down);
}

DMOD_TEST_STEP(parse_report_lifted_finger_is_not_down)
{
    const uint8_t entry[5] = { 0x10, 0x20, 0x00, 0x00, 0x02 };     /* finger 2, event 0 */
    uint8_t report[DMCST66XX_REPORT_SIZE];
    dmcst66xx_raw_contact_t c[5];
    make_report(report, 1, 0, entry);
    DMOD_TEST_EXPECT_EQ(dmcst66xx_parse_report(report, sizeof(report), c, 5), 1);
    DMOD_TEST_EXPECT_FALSE(c[0].down);
}

DMOD_TEST_STEP(parse_report_skips_the_keys)
{
    /* one key (its record comes first), then two fingers: the key is not a contact */
    const uint8_t entries[15] = { 0, 0, 0, 0, 0x11,
                                  0x10, 0x20, 0x30, 0x54, 0x11,         /* x 0x410, y 0x520 */
                                  0x01, 0x02, 0x03, 0x00, 0x22 };
    uint8_t report[DMCST66XX_REPORT_SIZE];
    dmcst66xx_raw_contact_t c[5];
    make_report(report, 2, 1, entries);
    DMOD_TEST_EXPECT_EQ(dmcst66xx_parse_report(report, sizeof(report), c, 5), 2);
    DMOD_TEST_EXPECT_EQ(c[0].x, 0x410);
    DMOD_TEST_EXPECT_EQ(c[0].y, 0x520);
    DMOD_TEST_EXPECT_EQ(c[0].id, 1);
    DMOD_TEST_EXPECT_EQ(c[1].x, 1);
    DMOD_TEST_EXPECT_EQ(c[1].y, 2);
    DMOD_TEST_EXPECT_EQ(c[1].id, 2);
}

DMOD_TEST_STEP(parse_report_limits_the_contacts)
{
    uint8_t entries[25];
    memset(entries, 0, sizeof(entries));
    for (unsigned i = 0; i < 5; i++)
        entries[5 * i + 4] = (uint8_t)(0x10 | i);
    uint8_t report[DMCST66XX_REPORT_SIZE];
    dmcst66xx_raw_contact_t c[5];
    make_report(report, 5, 0, entries);
    DMOD_TEST_EXPECT_EQ(dmcst66xx_parse_report(report, sizeof(report), c, 2), 2);
    DMOD_TEST_EXPECT_EQ(dmcst66xx_parse_report(report, sizeof(report), c, 5), 5);
    report[3] = 0x06;                                       /* the chip never reports more than five */
    DMOD_TEST_EXPECT_EQ(dmcst66xx_parse_report(report, sizeof(report), c, 5), -EBADMSG);
}

DMOD_TEST_STEP(parse_report_rejects_a_wrong_checksum)
{
    const uint8_t entry[5] = { 0xAB, 0x23, 0x40, 0x10, 0x13 };
    uint8_t report[DMCST66XX_REPORT_SIZE];
    dmcst66xx_raw_contact_t c[5];
    make_report(report, 1, 0, entry);
    report[4] ^= 0x01;                                      /* a bit of the record changed */
    DMOD_TEST_EXPECT_EQ(dmcst66xx_parse_report(report, sizeof(report), c, 5), -EBADMSG);
}

DMOD_TEST_STEP(map_point_identity_and_corners)
{
    dmcst66xx_config_t c = screen();
    uint16_t x, y;
    dmcst66xx_map_point(&c, 0, 0, &x, &y);
    DMOD_TEST_EXPECT_EQ(x, 0);
    DMOD_TEST_EXPECT_EQ(y, 0);
    dmcst66xx_map_point(&c, 239, 319, &x, &y);
    DMOD_TEST_EXPECT_EQ(x, 239);
    DMOD_TEST_EXPECT_EQ(y, 319);
    dmcst66xx_map_point(&c, 4000, 4000, &x, &y);        /* outside the range: clipped to the screen */
    DMOD_TEST_EXPECT_EQ(x, 239);
    DMOD_TEST_EXPECT_EQ(y, 319);
}

DMOD_TEST_STEP(map_point_scales)
{
    dmcst66xx_config_t c = screen();
    c.raw_width = 4096;
    c.raw_height = 4096;
    uint16_t x, y;
    dmcst66xx_map_point(&c, 4095, 4095, &x, &y);
    DMOD_TEST_EXPECT_EQ(x, 239);
    DMOD_TEST_EXPECT_EQ(y, 319);
    dmcst66xx_map_point(&c, 2048, 2048, &x, &y);
    DMOD_TEST_EXPECT_TRUE(x >= 119 && x <= 121);
    DMOD_TEST_EXPECT_TRUE(y >= 159 && y <= 161);
}

DMOD_TEST_STEP(map_point_mirrors)
{
    dmcst66xx_config_t c = screen();
    uint16_t x, y;
    c.mirror_x = true;
    dmcst66xx_map_point(&c, 0, 0, &x, &y);
    DMOD_TEST_EXPECT_EQ(x, 239);
    DMOD_TEST_EXPECT_EQ(y, 0);
    c.mirror_x = false;
    c.mirror_y = true;
    dmcst66xx_map_point(&c, 0, 0, &x, &y);
    DMOD_TEST_EXPECT_EQ(x, 0);
    DMOD_TEST_EXPECT_EQ(y, 319);
}

DMOD_TEST_STEP(map_point_swaps_the_axes)
{
    /* the chip is 240 x 320, the screen as shown 320 x 240 (rotated): x = raw y, y = raw x */
    dmcst66xx_config_t c = screen();
    c.width = 320;
    c.height = 240;
    c.swap_xy = true;
    uint16_t x, y;
    dmcst66xx_map_point(&c, 10, 300, &x, &y);
    DMOD_TEST_EXPECT_EQ(x, 300);
    DMOD_TEST_EXPECT_EQ(y, 10);
    dmcst66xx_map_point(&c, 239, 319, &x, &y);
    DMOD_TEST_EXPECT_EQ(x, 319);
    DMOD_TEST_EXPECT_EQ(y, 239);
}
