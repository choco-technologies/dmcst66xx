#define DMOD_ENABLE_REGISTRATION    ON
#include "dmod.h"
#include "dmcst66xx.h"
#include "dmdrvi.h"
#include "dmini.h"
#include "dmosi.h"
#include "dmgpio_types.h"
#include "dmi2c_types.h"
#include <errno.h>
#include <string.h>

/*
 * dmcst66xx: the touch panel with a CST66xx (Hynitron: CST3530, CST36xx, CST66xx, CST68xx) controller on I2C, as on the LILYGO T-Deck Pro
 * (V1.1). The protocol is the one of the vendor's driver (hyn_cst66xx.c): 32-bit register addresses, a report of 5 bytes per finger.
 * The CST328 is another protocol: dmcst328.
 *
 * The node follows the generic input interface (DMDRVI_IOCTL_INPUT_*) of dmdrvi: read() / GET_STATE give the contacts in screen
 * coordinates, WAIT_EVENT sleeps until they change. The chip is polled: through read() / GET_STATE when a client asks, and every
 * poll_ms while a client waits - nothing runs in the background.
 *
 * The wiring is done by dmdevfs: the sections of the touch panel, of its I2C bus (dmi2c) and of the reset line are in one
 * friends_group, and the roles of the friends (friend_role=...) tell this driver which node is which:
 *
 *   i2c        the dmi2c node the controller is on (required)
 *   reset      the reset line of the chip, low = reset (when the configuration says reset=yes)
 *
 * The interrupt line of the chip is not used (the driver polls).
 */

/* Magic set to "C66X" */
#define DMCST66XX_CONTEXT_MAGIC      0x43363658

/* 32-bit register addresses, most significant byte first; the low byte of some is a command, written without data */
#define REG_REPORT                  0xD0070000UL    /* the report: DMCST66XX_REPORT_SIZE bytes */
#define REG_REPORT_DONE             0xD00002ABUL    /* written once a report is read: the chip may make the next one */
#define REG_INFO                    0xD0030000UL    /* what the chip says about itself: INFO_SIZE bytes */
#define REG_LOW_POWER_OFF           0xD0000400UL    /* leave the low power I2C mode (the chip wakes up on the first access): twice */
#define REG_MODE_NORMAL_0           0xD0000000UL    /* the sequence that puts the chip in the normal mode */
#define REG_MODE_NORMAL_1           0xD0000C00UL
#define REG_MODE_NORMAL_2           0xD0000100UL

#define INFO_SIZE                   50U
#define INFO_TAG_0                  2U              /* the bytes 2 and 3 of the info are 0xCA 0xCA: what a CST66xx answers */
#define INFO_TAG_1                  3U
#define INFO_TAG                    0xCAU
#define INFO_CHIP                   0U              /* 4 bytes, little-endian */
#define INFO_RES_X                  28U             /* 16 bits, little-endian */
#define INFO_RES_Y                  30U
#define START_TRIES                 5U
#define CHECKSUM_SEED               0x55U

#define RAW_MAX                     4095U       /* 12 bits per axis */
#define DEFAULT_ADDRESS             0x1AU

typedef struct
{
    char   *path;                   /**< Node of the friend (NULL: not there) */
    void   *file;                   /**< Kept open once used */
} node_t;

typedef struct
{
    bool     used;
    uint8_t  id;
    uint8_t  pressure;
    uint16_t x, y;                  /**< Screen coordinates of the last report */
    uint64_t seen_ms;               /**< When the chip reported it last */
} tracked_t;

struct dmdrvi_context
{
    uint32_t              magic;
    dmcst66xx_config_t     config;
    uint8_t               instance;
    dmosi_mutex_t         lock;
    node_t                i2c;
    node_t                reset;
    bool                  reset_wired;      /**< The configuration says there is a reset line to wait for */
    bool                  started;          /**< The chip answered and was put in the normal mode */
    tracked_t             tracked[DMCST66XX_MAX_CONTACTS];
    dmdrvi_input_state_t  current;          /**< The last state computed */
    dmdrvi_input_state_t  handed;           /**< The last state a client got */
    bool                  unseen;           /**< current is not what the client has */
};

static bool is_valid_context(dmdrvi_context_t context)
{
    return context != NULL && context->magic == DMCST66XX_CONTEXT_MAGIC;
}

/* ---- API ---- */

dmod_dmcst66xx_api_declaration(1.0, bool, _validate_config, ( const dmcst66xx_config_t *config ))
{
    if (config == NULL || config->width == 0 || config->height == 0)
        return false;
    if (config->raw_width > RAW_MAX + 1U || config->raw_height > RAW_MAX + 1U)
        return false;
    if (config->address < 0x08 || config->address > 0x77)
        return false;
    return config->max_contacts >= 1 && config->max_contacts <= DMCST66XX_MAX_CONTACTS && config->poll_ms >= 1;
}

dmod_dmcst66xx_api_declaration(1.0, int, _parse_report, ( const uint8_t *report, size_t size, dmcst66xx_raw_contact_t *contacts, uint8_t max_contacts ))
{
    if (report == NULL || contacts == NULL || size < 4U)
        return -EBADMSG;
    if (report[2] != DMCST66XX_REPORT_TYPE_POS)
        return 0;                                       /* nothing new (zeros), or a gesture, or the proximity */
    uint8_t fingers = report[3] & 0x0FU, keys = (uint8_t)(report[3] >> 4);
    uint8_t entries = (uint8_t)(fingers + keys);
    if (entries > DMCST66XX_MAX_CONTACTS || size < 4U + 5U * (size_t)entries)
        return -EBADMSG;
    uint16_t sum = CHECKSUM_SEED;
    for (size_t i = 0; i < 5U * (size_t)entries; i++)
        sum = (uint16_t)(sum + report[4U + i]);
    if (sum != (uint16_t)(report[0] | (report[1] << 8)))
        return -EBADMSG;
    if (fingers > max_contacts)
        fingers = max_contacts;
    for (uint8_t i = 0; i < fingers; i++)
    {
        /* 5 bytes each: x low, y low, pressure, x high (low nibble) and y high (high nibble), the finger (low nibble) and the event
         * (high nibble, 0 = lifted). The keys come first. */
        const uint8_t *c = report + 4U + 5U * (size_t)(keys + i);
        contacts[i].x = (uint16_t)(c[0] | ((c[3] & 0x0FU) << 8));
        contacts[i].y = (uint16_t)(c[1] | ((c[3] & 0xF0U) << 4));
        contacts[i].pressure = c[2];
        contacts[i].id = (uint8_t)(c[4] & 0x0FU);
        contacts[i].down = (c[4] >> 4) != 0U;
    }
    return fingers;
}

dmod_dmcst66xx_api_declaration(1.0, void, _map_point, ( const dmcst66xx_config_t *config, uint16_t raw_x, uint16_t raw_y, uint16_t *x, uint16_t *y ))
{
    uint32_t rw = config->raw_width, rh = config->raw_height;
    uint32_t px = raw_x, py = raw_y;

    if (config->swap_xy)
    {
        uint32_t t = px; px = py; py = t;
        t = rw; rw = rh; rh = t;
    }
    if (px >= rw)
        px = rw - 1U;
    if (py >= rh)
        py = rh - 1U;
    if (config->mirror_x)
        px = rw - 1U - px;
    if (config->mirror_y)
        py = rh - 1U - py;
    /* Raw 0..rw-1 over 0..width-1 */
    uint32_t sx = rw > 1U ? (px * (uint32_t)(config->width - 1U) + (rw - 1U) / 2U) / (rw - 1U) : 0U;
    uint32_t sy = rh > 1U ? (py * (uint32_t)(config->height - 1U) + (rh - 1U) / 2U) / (rh - 1U) : 0U;
    *x = (uint16_t)sx;
    *y = (uint16_t)sy;
}

/* ---- Configuration ---- */

static bool string_to_switch(const char *text, bool fallback)
{
    if (text == NULL)
        return fallback;
    if (strcmp(text, "on") == 0 || strcmp(text, "yes") == 0 || strcmp(text, "true") == 0 || strcmp(text, "1") == 0)
        return true;
    if (strcmp(text, "off") == 0 || strcmp(text, "no") == 0 || strcmp(text, "false") == 0 || strcmp(text, "0") == 0)
        return false;
    return fallback;
}

static int read_config(dmdrvi_context_t context, dmini_context_t ini)
{
    dmcst66xx_config_t *c = &context->config;

    context->instance = (uint8_t)dmini_get_int(ini, NULL, "instance", 0);
    c->width        = (uint16_t)dmini_get_int(ini, NULL, "width", 240);
    c->height       = (uint16_t)dmini_get_int(ini, NULL, "height", 320);
    c->raw_width    = (uint16_t)dmini_get_int(ini, NULL, "raw_width", 0);
    c->raw_height   = (uint16_t)dmini_get_int(ini, NULL, "raw_height", 0);
    c->address      = (uint8_t)dmini_get_int(ini, NULL, "address", DEFAULT_ADDRESS);
    c->max_contacts = (uint8_t)dmini_get_int(ini, NULL, "max_contacts", DMCST66XX_MAX_CONTACTS);
    c->swap_xy      = string_to_switch(dmini_get_string(ini, NULL, "swap_xy", NULL), false);
    c->mirror_x     = string_to_switch(dmini_get_string(ini, NULL, "mirror_x", NULL), false);
    c->mirror_y     = string_to_switch(dmini_get_string(ini, NULL, "mirror_y", NULL), false);
    c->poll_ms      = (uint16_t)dmini_get_int(ini, NULL, "poll_ms", 10);
    c->release_ms   = (uint16_t)dmini_get_int(ini, NULL, "release_ms", 50);
    context->reset_wired = string_to_switch(dmini_get_string(ini, NULL, "reset", NULL), false);

    if (!dmcst66xx_validate_config(c))
    {
        DMOD_LOG_ERROR("Invalid touch configuration (screen %ux%u, address 0x%02X, %u contacts)\n",
                       c->width, c->height, c->address, (unsigned)c->max_contacts);
        return -EINVAL;
    }
    return 0;
}

/* ---- The nodes of the friends ---- */

static void node_close(node_t *node)
{
    if (node->file != NULL)
        Dmod_FileClose(node->file);
    node->file = NULL;
}

static void node_forget(node_t *node)
{
    node_close(node);
    Dmod_Free(node->path);
    node->path = NULL;
}

static void *node_file(node_t *node)
{
    if (node->file == NULL && node->path != NULL)
        node->file = Dmod_FileOpen(node->path, "r+");
    return node->file;
}

static int gpio_set(node_t *node, bool high)
{
    dmgpio_pins_state_t state = high ? dmgpio_pins_state_all_high : dmgpio_pins_state_all_low;
    void *file = node_file(node);
    if (file == NULL)
        return -ENODEV;
    if (Dmod_Ioctl(file, dmgpio_ioctl_cmd_set_pins_state, &state) != 0)
    {
        node_close(node);
        return -EIO;
    }
    return 0;
}

/* ---- The controller: 32-bit register addresses, high byte first ---- */

static int i2c_transfer(dmdrvi_context_t context, dmi2c_message_t *messages, size_t count)
{
    void *file = node_file(&context->i2c);
    dmi2c_transfer_t transfer = { messages, count };
    if (file == NULL)
        return -ENODEV;
    return Dmod_Ioctl(file, dmi2c_ioctl_cmd_transfer, &transfer);
}

/* The address, then the data with a repeated START */
static int reg_read(dmdrvi_context_t context, uint32_t reg, uint8_t *data, size_t size)
{
    uint8_t address[4] = { (uint8_t)(reg >> 24), (uint8_t)(reg >> 16), (uint8_t)(reg >> 8), (uint8_t)reg };
    dmi2c_message_t messages[2] = {
        { .address = context->config.address, .read = false, .data = address, .size = sizeof(address) },
        { .address = context->config.address, .read = true,  .data = data,    .size = size },
    };
    return i2c_transfer(context, messages, 2);
}

/* More of what the last register read started: the chip goes on from where it stopped */
static int read_on(dmdrvi_context_t context, uint8_t *data, size_t size)
{
    dmi2c_message_t message = { .address = context->config.address, .read = true, .data = data, .size = size };
    return i2c_transfer(context, &message, 1);
}

/* A command: the address alone is the whole message */
static int reg_command(dmdrvi_context_t context, uint32_t reg)
{
    uint8_t address[4] = { (uint8_t)(reg >> 24), (uint8_t)(reg >> 16), (uint8_t)(reg >> 8), (uint8_t)reg };
    dmi2c_message_t message = { .address = context->config.address, .read = false, .data = address, .size = sizeof(address) };
    return i2c_transfer(context, &message, 1);
}

static void tracked_clear(dmdrvi_context_t context)
{
    memset(context->tracked, 0, sizeof(context->tracked));
    memset(&context->current, 0, sizeof(context->current));
    memset(&context->handed, 0, sizeof(context->handed));
    context->unseen = false;
}

static void chip_reset(dmdrvi_context_t context)
{
    if (context->reset.path == NULL)
        return;
    (void)gpio_set(&context->reset, false);
    dmosi_thread_sleep(8);
    (void)gpio_set(&context->reset, true);
    dmosi_thread_sleep(50);                             /* the chip leaves its boot loader */
}

/* The chip sleeps (a low power I2C mode) when nothing touches it and does not answer then: the first access wakes it up - that one is
 * not answered, the second one, after a moment, is. */
static void chip_wake(dmdrvi_context_t context)
{
    (void)reg_command(context, REG_LOW_POWER_OFF);
    dmosi_thread_sleep(1);
    (void)reg_command(context, REG_LOW_POWER_OFF);
}

/* A read that wakes the chip up when it does not answer */
static int reg_read_awake(dmdrvi_context_t context, uint32_t reg, uint8_t *data, size_t size)
{
    int ret = reg_read(context, reg, data, size);
    if (ret == 0)
        return 0;
    chip_wake(context);
    return reg_read(context, reg, data, size);
}

/*
 * The report: the header and the first entry (9 bytes) from the register, the other entries (5 bytes each, as many as the header says)
 * as a continuation. A longer read at once is not answered by the chip.
 */
static int report_read(dmdrvi_context_t context, uint8_t *report)
{
    int ret = reg_read_awake(context, REG_REPORT, report, DMCST66XX_REPORT_HEAD_SIZE);
    if (ret != 0 || report[2] != DMCST66XX_REPORT_TYPE_POS)
        return ret;
    unsigned entries = (report[3] & 0x0FU) + (unsigned)(report[3] >> 4);
    if (entries > 1U && entries <= DMCST66XX_MAX_CONTACTS)
        ret = read_on(context, report + DMCST66XX_REPORT_HEAD_SIZE, 5U * (entries - 1U));
    return ret;
}

/* The normal mode (the chip wakes up on the first access, so the first command may fail), then what the chip says about itself */
static int chip_normal_mode(dmdrvi_context_t context, uint8_t *info)
{
    chip_wake(context);
    int ret = reg_command(context, REG_MODE_NORMAL_0);
    if (ret == 0)
        ret = reg_command(context, REG_MODE_NORMAL_1);
    if (ret == 0)
        ret = reg_command(context, REG_MODE_NORMAL_2);
    if (ret == 0)
        ret = reg_read(context, REG_INFO, info, INFO_SIZE);
    if (ret == 0 && (info[INFO_TAG_0] != INFO_TAG || info[INFO_TAG_1] != INFO_TAG))
        ret = -ENODEV;                                  /* it answers, but it is not a CST66xx */
    return ret;
}

/* Reset the chip (when there is a line for it), put it in the normal mode and read what it says about itself */
static int chip_start(dmdrvi_context_t context)
{
    dmcst66xx_config_t *c = &context->config;
    uint8_t info[INFO_SIZE];
    int ret = -ENODEV;

    chip_reset(context);
    for (unsigned try = 0; try < START_TRIES && ret != 0; try++)
        ret = chip_normal_mode(context, info);
    if (ret != 0)
    {
        DMOD_LOG_ERROR("CST66xx: no answer at 0x%02X (%d)\n", c->address, ret);
        return ret;
    }
    uint32_t id = (uint32_t)info[INFO_CHIP] | ((uint32_t)info[INFO_CHIP + 1] << 8) | ((uint32_t)info[INFO_CHIP + 2] << 16)
                | ((uint32_t)info[INFO_CHIP + 3] << 24);
    uint16_t res_x = (uint16_t)(info[INFO_RES_X] | (info[INFO_RES_X + 1] << 8));
    uint16_t res_y = (uint16_t)(info[INFO_RES_Y] | (info[INFO_RES_Y + 1] << 8));
    DMOD_LOG_INFO("CST66xx: chip 0x%08X, resolution %ux%u\n", (unsigned)id, res_x, res_y);
    /* The range of the chip: the configured one, else the one it reports (the screen's when it reports nothing sensible) */
    if (c->raw_width == 0)
        c->raw_width = (res_x != 0 && res_x <= RAW_MAX + 1U) ? res_x : c->width;
    if (c->raw_height == 0)
        c->raw_height = (res_y != 0 && res_y <= RAW_MAX + 1U) ? res_y : c->height;
    return 0;
}

static void try_start(dmdrvi_context_t context)
{
    if (context->started || context->i2c.path == NULL || (context->reset_wired && context->reset.path == NULL))
        return;
    dmosi_mutex_lock(context->lock);
    if (!context->started && chip_start(context) == 0)
    {
        context->started = true;
        tracked_clear(context);
        DMOD_LOG_INFO("CST66xx %ux%u started (range %ux%u)\n", context->config.width, context->config.height,
                      context->config.raw_width, context->config.raw_height);
    }
    dmosi_mutex_unlock(context->lock);
}

/* ---- The state of the contacts ---- */

static int tracked_find(dmdrvi_context_t context, uint8_t id)
{
    for (int i = 0; i < DMCST66XX_MAX_CONTACTS; i++)
    {
        if (context->tracked[i].used && context->tracked[i].id == id)
            return i;
    }
    return -1;
}

static int tracked_free(dmdrvi_context_t context)
{
    for (int i = 0; i < DMCST66XX_MAX_CONTACTS; i++)
    {
        if (!context->tracked[i].used)
            return i;
    }
    return -1;
}

static void state_add(dmdrvi_input_state_t *state, const tracked_t *t, dmdrvi_input_contact_event_t event)
{
    dmdrvi_input_contact_t *c = &state->contacts[state->contact_count++];
    c->x = t->x;
    c->y = t->y;
    c->id = t->id;
    c->event = (uint8_t)event;
    c->pressure = t->pressure;
    c->size = 0;
}

/* Ask the chip what is touched and compute the new state (the lock is held); a state that was not handed yet is kept as it is */
static int poll_once(dmdrvi_context_t context)
{
    uint8_t report[DMCST66XX_REPORT_SIZE] = { 0 };
    dmcst66xx_raw_contact_t raw[DMCST66XX_MAX_CONTACTS];
    dmdrvi_input_state_t next;
    bool present[DMCST66XX_MAX_CONTACTS] = { false };
    uint8_t count = 0;

    if (!context->started)
        return -ENODEV;
    if (context->unseen)
        return 0;
    int ret = report_read(context, report);
    if (ret != 0)
        return ret;
    int decoded = dmcst66xx_parse_report(report, sizeof(report), raw, context->config.max_contacts);
    if (decoded < 0)
        return 0;                                       /* a damaged report (the chip was updating it): the state stays */
    count = (uint8_t)decoded;
    if (report[2] == DMCST66XX_REPORT_TYPE_POS)
        (void)reg_command(context, REG_REPORT_DONE);    /* ready for the next report */
    uint64_t now = Dmod_GetUptime();

    memset(&next, 0, sizeof(next));
    for (uint8_t i = 0; i < count; i++)
    {
        if (!raw[i].down)
            continue;
        int slot = tracked_find(context, raw[i].id);
        dmdrvi_input_contact_event_t event = slot >= 0 ? DMDRVI_INPUT_CONTACT_MOVE : DMDRVI_INPUT_CONTACT_DOWN;
        if (slot < 0)
            slot = tracked_free(context);
        if (slot < 0)
            continue;
        tracked_t *t = &context->tracked[slot];
        t->used = true;
        t->id = raw[i].id;
        t->pressure = raw[i].pressure;
        t->seen_ms = now;
        dmcst66xx_map_point(&context->config, raw[i].x, raw[i].y, &t->x, &t->y);
        present[slot] = true;
        state_add(&next, t, event);
    }
    for (int i = 0; i < DMCST66XX_MAX_CONTACTS; i++)
    {
        tracked_t *t = &context->tracked[i];
        if (!t->used || present[i])
            continue;
        if (now - t->seen_ms >= context->config.release_ms)
        {
            state_add(&next, t, DMDRVI_INPUT_CONTACT_UP);
            t->used = false;
        }
        else
        {
            state_add(&next, t, DMDRVI_INPUT_CONTACT_MOVE);     /* no news yet: it stays where it was */
        }
    }
    if (!dmdrvi_input_state_equal(&next, &context->handed))
        context->unseen = true;
    context->current = next;
    return 0;
}

/* The state for a client: polled when nothing is waiting to be handed out */
static int get_state(dmdrvi_context_t context, dmdrvi_input_state_t *out)
{
    dmosi_mutex_lock(context->lock);
    int ret = poll_once(context);
    if (ret == 0)
    {
        *out = context->current;
        context->handed = context->current;
        context->unseen = false;
    }
    dmosi_mutex_unlock(context->lock);
    return ret;
}

static int wait_event(dmdrvi_context_t context, const uint32_t *timeout_ms)
{
    uint64_t start = Dmod_GetUptime();

    for (;;)
    {
        dmosi_mutex_lock(context->lock);
        int ret = poll_once(context);
        bool changed = context->unseen;
        dmosi_mutex_unlock(context->lock);
        if (ret != 0)
            return ret;
        if (changed)
            return 0;
        if (timeout_ms != NULL && Dmod_GetUptime() - start >= *timeout_ms)
            return -ETIMEDOUT;
        dmosi_thread_sleep(context->config.poll_ms);
    }
}

/* ---- DMOD lifecycle ---- */

int dmod_init(const Dmod_Config_t *Config)
{
    (void)Config;
    DMOD_LOG_INFO("DMCST66XX module initialized\n");
    return 0;
}

int dmod_deinit(void)
{
    DMOD_LOG_INFO("DMCST66XX module deinitialized\n");
    return 0;
}

/* ---- DMDRVI interface ---- */

dmod_dmdrvi_dif_api_declaration(2.0, dmcst66xx, dmdrvi_context_t, _create, ( dmini_context_t config, dmdrvi_dev_num_t* dev_num ))
{
    if (config == NULL || dev_num == NULL)
        return NULL;
    dmdrvi_context_t context = Dmod_Malloc(sizeof(struct dmdrvi_context));
    if (context == NULL)
        return NULL;
    memset(context, 0, sizeof(*context));
    context->magic = DMCST66XX_CONTEXT_MAGIC;

    if (read_config(context, config) != 0)
        goto fail;
    context->lock = dmosi_mutex_create(false);
    if (context->lock == NULL)
    {
        DMOD_LOG_ERROR("CST66xx: out of memory\n");
        goto fail;
    }
    dev_num->flags = DMDRVI_NUM_MAJOR;
    dev_num->major = (dmdrvi_dev_id_t)context->instance;
    return context;

fail:
    context->magic = 0;
    Dmod_Free(context);
    return NULL;
}

dmod_dmdrvi_dif_api_declaration(2.0, dmcst66xx, void, _free, ( dmdrvi_context_t context ))
{
    if (!is_valid_context(context))
        return;
    /* The nodes of the friends are not touched: dmdevfs tears them down before this call, in reverse order of the sections */
    Dmod_Free(context->i2c.path);
    Dmod_Free(context->reset.path);
    dmosi_mutex_destroy(context->lock);
    context->magic = 0;
    Dmod_Free(context);
}

dmod_dmdrvi_dif_api_declaration(2.0, dmcst66xx, void, _friend_changed, ( dmdrvi_context_t context, const dmdrvi_friend_info_t* info ))
{
    if (!is_valid_context(context) || info == NULL || info->friend_role == NULL)
        return;
    node_t *node = strcmp(info->friend_role, "i2c") == 0 ? &context->i2c
                 : strcmp(info->friend_role, "reset") == 0 ? &context->reset : NULL;
    if (node == NULL)
        return;
    node_forget(node);
    if (node == &context->i2c)
        context->started = false;                       /* a new bus: the chip is started again */
    if (info->state != dmdrvi_dev_state_ready || info->node_path == NULL)
        return;
    node->path = Dmod_StrDup(info->node_path);
    if (node->path == NULL)
    {
        DMOD_LOG_ERROR("CST66xx: out of memory for the path of a friend\n");
        return;
    }
    try_start(context);
}

dmod_dmdrvi_dif_api_declaration(2.0, dmcst66xx, void*, _open, ( dmdrvi_context_t context, int flags, const dmdrvi_dev_num_t* dev_num ))
{
    return is_valid_context(context) ? context : NULL;
}

dmod_dmdrvi_dif_api_declaration(2.0, dmcst66xx, void, _close, ( dmdrvi_context_t context, void* handle ))
{
}

/* read(): the state of the contacts, one dmdrvi_input_state_t */
dmod_dmdrvi_dif_api_declaration(2.0, dmcst66xx, dmdrvi_ssize_t, _read, ( dmdrvi_context_t context, void* handle, void* buffer, size_t size, dmdrvi_offset_t offset ))
{
    if (!is_valid_context(context) || buffer == NULL || size < sizeof(dmdrvi_input_state_t))
        return -EINVAL;
    int ret = get_state(context, (dmdrvi_input_state_t *)buffer);
    return ret != 0 ? ret : (dmdrvi_ssize_t)sizeof(dmdrvi_input_state_t);
}

dmod_dmdrvi_dif_api_declaration(2.0, dmcst66xx, dmdrvi_ssize_t, _write, ( dmdrvi_context_t context, void* handle, const void* buffer, size_t size, dmdrvi_offset_t offset ))
{
    return -ENOTSUP;
}

dmod_dmdrvi_dif_api_declaration(2.0, dmcst66xx, int, _ioctl, ( dmdrvi_context_t context, void* handle, int command, void* arg ))
{
    if (!is_valid_context(context))
        return -EINVAL;

    switch (command)
    {
        case DMDRVI_IOCTL_INPUT_GET_INFO:
        {
            if (arg == NULL)
                return -EINVAL;
            dmdrvi_input_info_t *info = (dmdrvi_input_info_t *)arg;
            memset(info, 0, sizeof(*info));
            memcpy(info->name, "CST66xx", 7);
            info->type = DMDRVI_INPUT_TYPE_TOUCHSCREEN;
            info->capabilities = DMDRVI_INPUT_CAP_CONTACTS | DMDRVI_INPUT_CAP_PRESSURE;
            info->width = context->config.width;
            info->height = context->config.height;
            info->max_contacts = context->config.max_contacts;
            return 0;
        }
        case DMDRVI_IOCTL_INPUT_GET_STATE:
            return arg == NULL ? -EINVAL : get_state(context, (dmdrvi_input_state_t *)arg);
        case DMDRVI_IOCTL_INPUT_WAIT_EVENT:
            return wait_event(context, (const uint32_t *)arg);
        case dmcst66xx_ioctl_cmd_get_config:
            if (arg == NULL)
                return -EINVAL;
            *(dmcst66xx_config_t *)arg = context->config;
            return 0;
        default:
            return -ENOTTY;
    }
}

dmod_dmdrvi_dif_api_declaration(2.0, dmcst66xx, int, _flush, ( dmdrvi_context_t context, void* handle ))
{
    return is_valid_context(context) ? 0 : -EINVAL;
}

dmod_dmdrvi_dif_api_declaration(2.0, dmcst66xx, int, _stat, ( dmdrvi_context_t context, const char* path, dmdrvi_stat_t* stat ))
{
    if (!is_valid_context(context) || stat == NULL)
        return -EINVAL;
    stat->size = (dmdrvi_size_t)sizeof(dmdrvi_input_state_t);
    stat->mode = 0444;
    return 0;
}
