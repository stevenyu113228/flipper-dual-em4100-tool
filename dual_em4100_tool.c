/**
 * @file dual_em4100_tool.c
 * @brief Read, save and emulate dual-frame (128-bit) EM4100 cards.
 *
 * Some access cards are T5577 chips configured with MAXBLOCK=4: instead of a
 * standard EM4100's single 64-bit frame, they broadcast TWO 64-bit frames
 * back-to-back (128 bits), looping. Flipper's built-in EM4100 decoder only has a
 * 64-bit view and reports just the first frame, so a normal clone is missing the
 * second half and the door stays shut.
 *
 * This tool captures the raw LF waveform, runs a custom 128-bit Manchester
 * decoder, auto-detects single vs dual frame, lets you save / rename / delete /
 * add-by-hand cards, and emulates the FULL signal. Built on ViewDispatcher with
 * the stock submenu / widget / popup / text_input / byte_input modules.
 */

#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_rfid.h>

#include <gui/gui.h>
#include <gui/view_dispatcher.h>
#include <gui/modules/submenu.h>
#include <gui/modules/widget.h>
#include <gui/modules/popup.h>
#include <gui/modules/text_input.h>
#include <gui/modules/byte_input.h>
#include <storage/storage.h>
#include <dialogs/dialogs.h>
#include <notification/notification.h>
#include <notification/notification_messages.h>
#include <toolbox/manchester_decoder.h>
#include <toolbox/path.h>
#include <lfrfid/tools/t5577.h>

#define TAG "DualEM4100Tool"

// ---- Decode timing (RF/64). Capture duration is microseconds.
#define DE_SHORT_US   256
#define DE_LONG_US    512
#define DE_JITTER_US  120
#define DE_SHORT_LOW  (DE_SHORT_US - DE_JITTER_US)
#define DE_SHORT_HIGH (DE_SHORT_US + DE_JITTER_US)
#define DE_LONG_LOW   (DE_LONG_US - DE_JITTER_US)
#define DE_LONG_HIGH  (DE_LONG_US + DE_JITTER_US)

// ---- Emulation timing: bit = 64 carrier clocks, two 32-clock half-cells.
#define DE_HALF_BIT   32u
#define DE_HALF_CELLS 256u
#define DE_LOOP_MAX   128u
#define DE_REPEAT     8u
#define DE_BUFFER_MAX (DE_LOOP_MAX * DE_REPEAT)

#define DE_READ_VALIDATE   3
#define DE_READ_TIMEOUT_MS 6000
#define DE_FOLDER          EXT_PATH("apps_data/dual_em4100")
#define DE_EXTENSION       ".dem4100"
#define DE_NAME_LEN        32

// ViewDispatcher view ids
typedef enum {
    DeViewSubmenu,
    DeViewPopup,
    DeViewWidget,
    DeViewTextInput,
    DeViewByteInput,
} DeViewId;

// Submenu item ids
typedef enum {
    DeMenuRead,
    DeMenuSaved,
    DeMenuAddManual,
    DeMenuAbout,
} DeMenuId;

// What the card-detail menu action should do after a text/byte input.
typedef enum {
    DeInputNone,
    DeInputSaveName, // naming a freshly read card
    DeInputRename, // renaming an existing saved card
    DeInputAddFrames, // byte_input for manual add
} DePendingInput;

typedef struct {
    Gui* gui;
    ViewDispatcher* view_dispatcher;
    Submenu* submenu;
    Popup* popup;
    Widget* widget;
    TextInput* text_input;
    ByteInput* byte_input;
    Storage* storage;
    DialogsApp* dialogs;
    NotificationApp* notifications;

    // Current card: two 64-bit encoded EM4100 frames, MSB first.
    uint64_t frame1;
    uint64_t frame2;
    uint8_t segments; // 1 or 2

    // Read worker
    FuriThread* read_thread;
    FuriTimer* read_timer;
    FuriStreamBuffer* capture_stream;
    volatile bool read_running;
    volatile bool read_found;
    uint32_t isr_pulse;
    bool isr_have_pulse;

    // Emulation
    uint32_t* em_duration;
    uint32_t* em_pulse;
    size_t em_size;
    FuriStreamBuffer* em_stream;
    bool emulating;

    // T5577 write worker
    FuriThread* write_thread;
    FuriTimer* write_timer;
    volatile bool write_running;
    volatile int write_result; // -1 in progress, 0 ok, 1 mismatch, 2 no read

    // Inputs
    DePendingInput pending;
    char name_buf[DE_NAME_LEN];
    uint8_t byte_buf[16]; // 8 bytes frame1 + 8 bytes frame2
    FuriString* loaded_path; // path of the saved card currently open in detail

    DeViewId current_view; // tracked so Back knows where to return
    bool submenu_is_detail; // false = main menu, true = card-detail menu
    char id_text[24];
} DeApp;

typedef struct {
    uint32_t pulse;
    uint32_t period;
} DeCaptureEvent;

// ---- Forward declarations & shared navigation helper ------------------------

static void de_show_menu(DeApp* app);
static void de_show_read_result(DeApp* app);
static void de_show_card_detail(DeApp* app);
static void de_start_read(DeApp* app);
static void de_text_input_done(void* context);

// Switch view and remember it so the Back handler knows where to return.
static void de_switch(DeApp* app, DeViewId view) {
    app->current_view = view;
    view_dispatcher_switch_to_view(app->view_dispatcher, view);
}

// ============================================================================
// EM4100 frame helpers
// ============================================================================

static void de_frame_to_id(uint64_t frame, uint8_t id[5]) {
    uint8_t nibbles[10];
    for(uint8_t r = 0; r < 10; r++) {
        uint8_t shift = 63 - 9 - (r * 5);
        nibbles[r] = (frame >> (shift - 3)) & 0xF;
    }
    for(uint8_t i = 0; i < 5; i++) {
        id[i] = (nibbles[i * 2] << 4) | nibbles[i * 2 + 1];
    }
}

// strict=true requires full row+column parity (a real frame1). strict=false
// only checks header + stop bit (frame2 may carry a broken row parity).
static bool de_frame_valid(uint64_t frame, bool strict) {
    if((frame >> 55) != 0x1FF) return false;
    if(frame & 1) return false;
    if(!strict) return true;
    for(uint8_t r = 0; r < 10; r++) {
        uint8_t shift = 63 - 9 - (r * 5);
        uint8_t sum = 0;
        for(uint8_t b = 0; b < 5; b++) sum += (frame >> (shift - b)) & 1;
        if(sum & 1) return false;
    }
    for(uint8_t c = 0; c < 4; c++) {
        uint8_t sum = 0;
        for(uint8_t r = 0; r < 11; r++) sum += (frame >> (63 - 9 - (r * 5) - c)) & 1;
        if(sum & 1) return false;
    }
    return true;
}

static void de_update_id_text(DeApp* app) {
    uint8_t id[5];
    de_frame_to_id(app->frame1, id);
    snprintf(
        app->id_text, sizeof(app->id_text), "%02X %02X %02X %02X %02X",
        id[0], id[1], id[2], id[3], id[4]);
}

// ============================================================================
// Read: capture ISR + decoder thread
// ============================================================================

// Capture fires twice per period: level=true gives the high-level duration
// (pulse), level=false gives the full period. We pair them and forward one event
// per period; the decoder splits each into a high half and a low half.
static void de_capture_isr(bool level, uint32_t duration, void* context) {
    DeApp* app = context;
    if(level) {
        app->isr_pulse = duration;
        app->isr_have_pulse = true;
    } else if(app->isr_have_pulse) {
        DeCaptureEvent ev = {.pulse = app->isr_pulse, .period = duration};
        app->isr_have_pulse = false;
        furi_stream_buffer_send(app->capture_stream, &ev, sizeof(ev), 0);
    }
}

typedef struct {
    ManchesterState m_state;
    uint64_t hi;
    uint64_t lo;
} DeDecoder;

static void de_decoder_reset(DeDecoder* d) {
    manchester_advance(d->m_state, ManchesterEventReset, &d->m_state, NULL);
    d->hi = 0;
    d->lo = 0;
}

static bool de_decoder_half(
    DeDecoder* d, bool level, uint32_t duration, uint64_t* f1, uint64_t* f2, uint8_t* segs) {
    ManchesterEvent event;
    if(duration > DE_SHORT_LOW && duration < DE_SHORT_HIGH) {
        event = level ? ManchesterEventShortLow : ManchesterEventShortHigh;
    } else if(duration > DE_LONG_LOW && duration < DE_LONG_HIGH) {
        event = level ? ManchesterEventLongLow : ManchesterEventLongHigh;
    } else {
        return false;
    }
    bool data;
    if(!manchester_advance(d->m_state, event, &d->m_state, &data)) return false;
    bool carry = (d->lo >> 63) & 1;
    d->hi = (d->hi << 1) | carry;
    d->lo = (d->lo << 1) | (data ? 1 : 0);
    if(!de_frame_valid(d->hi, true)) return false;
    if(de_frame_valid(d->lo, false)) {
        *f1 = d->hi;
        *f2 = d->lo;
        *segs = 2;
    } else {
        *f1 = d->hi;
        *f2 = d->hi;
        *segs = 1;
    }
    return true;
}

static bool de_decoder_feed(
    DeDecoder* d, uint32_t pulse, uint32_t period, uint64_t* f1, uint64_t* f2, uint8_t* segs) {
    bool found = de_decoder_half(d, true, pulse, f1, f2, segs);
    if(!found) {
        uint32_t low = (period > pulse) ? (period - pulse) : 0;
        found = de_decoder_half(d, false, low, f1, f2, segs);
    }
    return found;
}

// Energise the field, capture and decode until a card is confirmed (the same
// result seen DE_READ_VALIDATE times) or timeout_ms elapses. Returns true and
// fills out_f1/out_f2/out_segs on success. Used by both the reader and the
// post-write verifier. `running` lets the caller abort (NULL = run to timeout).
static bool de_capture_decode(
    DeApp* app,
    uint32_t timeout_ms,
    volatile bool* running,
    uint64_t* out_f1,
    uint64_t* out_f2,
    uint8_t* out_segs) {
    if(app->capture_stream) furi_stream_buffer_free(app->capture_stream);
    app->capture_stream =
        furi_stream_buffer_alloc(sizeof(DeCaptureEvent) * 64, sizeof(DeCaptureEvent));

    app->isr_have_pulse = false;
    furi_hal_rfid_tim_read_start(125000.0f, 0.5f);
    furi_delay_ms(1500);
    furi_hal_rfid_tim_read_capture_start(de_capture_isr, app);

    DeDecoder dec;
    de_decoder_reset(&dec);
    uint64_t last_f1 = 0, last_f2 = 0;
    uint8_t last_segs = 0, streak = 0;
    uint32_t start = furi_get_tick();
    bool found = false;

    DeCaptureEvent ev;
    while(running == NULL || *running) {
        if(furi_stream_buffer_receive(app->capture_stream, &ev, sizeof(ev), 50) == sizeof(ev)) {
            uint64_t f1, f2;
            uint8_t segs;
            if(de_decoder_feed(&dec, ev.pulse, ev.period, &f1, &f2, &segs)) {
                if(f1 == last_f1 && f2 == last_f2 && segs == last_segs) {
                    if(++streak >= DE_READ_VALIDATE) {
                        *out_f1 = f1;
                        *out_f2 = f2;
                        *out_segs = segs;
                        found = true;
                        break;
                    }
                } else {
                    last_f1 = f1;
                    last_f2 = f2;
                    last_segs = segs;
                    streak = 1;
                }
            }
        }
        if(furi_get_tick() - start > timeout_ms) break;
    }

    furi_hal_rfid_tim_read_capture_stop();
    furi_hal_rfid_tim_read_stop();
    return found;
}

static int32_t de_read_thread(void* ctx) {
    DeApp* app = ctx;
    uint64_t f1, f2;
    uint8_t segs;
    if(de_capture_decode(app, DE_READ_TIMEOUT_MS, &app->read_running, &f1, &f2, &segs)) {
        app->frame1 = f1;
        app->frame2 = f2;
        app->segments = segs;
        // Publish the data before the read_found flag the UI timer polls, so the
        // timer never observes read_found==true with stale frame fields.
        __DMB();
        app->read_found = true;
    }
    return 0;
}

// ============================================================================
// Emulation
// ============================================================================

static void de_build_waveform(DeApp* app) {
    uint64_t f1 = app->frame1;
    uint64_t f2 = (app->segments == 2) ? app->frame2 : app->frame1;
    uint64_t frames[2] = {f1, f2};

    static uint8_t levels[DE_HALF_CELLS];
    size_t n = 0;
    for(uint8_t f = 0; f < 2; f++) {
        for(uint8_t i = 0; i < 64; i++) {
            bool bit = (frames[f] >> (63 - i)) & 1;
            levels[n++] = bit ? 1 : 0;
            levels[n++] = bit ? 0 : 1;
        }
    }
    size_t start = 0;
    for(size_t i = 0; i < n; i++) {
        if(levels[i] && !levels[(i + n - 1) % n]) {
            start = i;
            break;
        }
    }
    size_t count = 0, i = 0;
    while(i < n && count < DE_LOOP_MAX) {
        uint32_t high = 0;
        while(i < n && levels[(start + i) % n]) {
            high++;
            i++;
        }
        uint32_t low = 0;
        while(i < n && !levels[(start + i) % n]) {
            low++;
            i++;
        }
        uint32_t period = (high + low) * DE_HALF_BIT;
        app->em_duration[count] = period - 1;
        app->em_pulse[count] = high * DE_HALF_BIT;
        count++;
    }
    size_t loop = count;
    for(uint8_t r = 1; r < DE_REPEAT && (count + loop) <= DE_BUFFER_MAX; r++) {
        for(size_t k = 0; k < loop; k++) {
            app->em_duration[count] = app->em_duration[k];
            app->em_pulse[count] = app->em_pulse[k];
            count++;
        }
    }
    app->em_size = count;
}

static void de_emulate_isr(bool half, void* context) {
    DeApp* app = context;
    uint32_t flag = half ? 0 : 1;
    furi_stream_buffer_send(app->em_stream, &flag, sizeof(uint32_t), 0);
}

static void de_emulate_start(DeApp* app) {
    if(app->emulating) return;
    de_build_waveform(app);
    furi_hal_rfid_tim_emulate_dma_start(
        app->em_duration, app->em_pulse, app->em_size, de_emulate_isr, app);
    app->emulating = true;
}

static void de_emulate_stop(DeApp* app) {
    if(!app->emulating) return;
    furi_hal_rfid_tim_emulate_dma_stop();
    app->emulating = false;
}

// ============================================================================
// Write to a blank T5577 card (+ read-back verify)
// ============================================================================

// Config word for dual-frame cards. The reference card's own T5577 dump uses
// 0x00148C82 (Manchester | RF/64 | MAXBLOCK=4, plus a few low bits the issuer
// set). We replicate it verbatim so the cloned card is bit-identical to the
// original, maximising reader compatibility. The standard value 0x00148080
// (no extra low bits) also works for plain Manchester EM4100.
#define DE_DUAL_CONFIG 0x00148C82UL

// Fill the T5577 block layout. Dual-frame -> MAXBLOCK=4 / 5 blocks; single-frame
// -> standard EM4100 MAXBLOCK=2 / 3 blocks.
static void de_fill_t5577(DeApp* app, LFRFIDT5577* tag) {
    if(app->segments == 2) {
        tag->block[0] = DE_DUAL_CONFIG;
        tag->block[1] = app->frame1 >> 32;
        tag->block[2] = app->frame1 & 0xFFFFFFFF;
        tag->block[3] = app->frame2 >> 32;
        tag->block[4] = app->frame2 & 0xFFFFFFFF;
        tag->blocks_to_write = 5;
    } else {
        tag->block[0] = LFRFID_T5577_MODULATION_MANCHESTER | LFRFID_T5577_BITRATE_RF_64 |
                        (2 << LFRFID_T5577_MAXBLOCK_SHIFT);
        tag->block[1] = app->frame1 >> 32;
        tag->block[2] = app->frame1 & 0xFFFFFFFF;
        tag->blocks_to_write = 3;
    }
    tag->mask = 0;
}

static int32_t de_write_thread(void* ctx) {
    DeApp* app = ctx;

    // Let the RF hardware settle before the bit-banged write. The built-in
    // writer does the same `furi_delay_ms(5)` "halt" before every t5577_write();
    // without it the field/timer left over from stopping emulation corrupts the
    // microsecond write timing (0 bits get written as 1, blocks come out wrong).
    furi_delay_ms(5);

    // 1. Write the blank card (blocking ~150ms, manages the RF field itself).
    LFRFIDT5577 tag = {0};
    de_fill_t5577(app, &tag);
    t5577_write(&tag);

    // 2. Read it back and compare (reuses the reader's capture+decode).
    uint64_t f1 = 0, f2 = 0;
    uint8_t segs = 0;
    bool got = de_capture_decode(app, 2000, &app->write_running, &f1, &f2, &segs);
    if(!got) {
        app->write_result = 2; // no read
    } else if(f1 == app->frame1 && segs == app->segments && (segs == 1 || f2 == app->frame2)) {
        app->write_result = 0; // success
    } else {
        app->write_result = 1; // mismatch
    }
    return 0;
}

static void de_write_finish(DeApp* app) {
    app->write_running = false;
    if(app->write_thread) {
        furi_thread_join(app->write_thread);
        furi_thread_free(app->write_thread);
        app->write_thread = NULL;
    }
}

static void de_write_timer_cb(void* context) {
    DeApp* app = context;
    // Nothing to poll (thread already reaped) -> stop and bail.
    if(!app->write_thread) {
        if(app->write_timer) furi_timer_stop(app->write_timer);
        return;
    }
    if(furi_thread_get_state(app->write_thread) != FuriThreadStateStopped) {
        return; // still writing/verifying
    }
    // worker done
    furi_timer_stop(app->write_timer);
    de_write_finish(app);

    popup_reset(app->popup);
    if(app->write_result == 0) {
        notification_message(app->notifications, &sequence_success);
        notification_message(app->notifications, &sequence_single_vibro);
        popup_set_header(app->popup, "Write OK!", 64, 18, AlignCenter, AlignTop);
        popup_set_text(app->popup, "Card written and\nverified.", 64, 34, AlignCenter, AlignTop);
    } else {
        notification_message(app->notifications, &sequence_error);
        popup_set_header(app->popup, "Write failed", 64, 18, AlignCenter, AlignTop);
        popup_set_text(
            app->popup,
            app->write_result == 2 ? "No card read back.\nReposition & retry." :
                                     "Data mismatch.\nReposition & retry.",
            64,
            34,
            AlignCenter,
            AlignTop);
    }
    de_switch(app, DeViewPopup);
}

static void de_start_write(DeApp* app) {
    de_emulate_stop(app); // mutual exclusion: free the RF/timer
    app->write_running = true;
    app->write_result = -1;
    app->write_thread = furi_thread_alloc_ex("DeWrite", 2048, de_write_thread, app);
    furi_thread_start(app->write_thread);

    popup_reset(app->popup);
    popup_set_header(app->popup, "Writing T5577...", 64, 18, AlignCenter, AlignTop);
    popup_set_text(
        app->popup, "Hold a blank T5577\non the back", 64, 34, AlignCenter, AlignTop);
    de_switch(app, DeViewPopup);

    furi_timer_start(app->write_timer, furi_ms_to_ticks(150));
}

// ============================================================================
// Save / Load / Delete
// ============================================================================

static void de_card_path(DeApp* app, const char* name, FuriString* out) {
    furi_string_printf(out, "%s/%s%s", DE_FOLDER, name, DE_EXTENSION);
    UNUSED(app);
}

static bool de_save_as(DeApp* app, const char* name) {
    storage_common_mkdir(app->storage, DE_FOLDER);
    FuriString* path = furi_string_alloc();
    de_card_path(app, name, path);

    File* file = storage_file_alloc(app->storage);
    bool ok = false;
    if(storage_file_open(file, furi_string_get_cstr(path), FSAM_WRITE, FSOM_CREATE_ALWAYS)) {
        FuriString* c = furi_string_alloc();
        furi_string_printf(
            c,
            "Filetype: Dual EM4100\nVersion: 1\nFrame1: %016llX\nFrame2: %016llX\nSegments: %u\n",
            (unsigned long long)app->frame1,
            (unsigned long long)app->frame2,
            app->segments);
        size_t len = furi_string_size(c);
        ok = storage_file_write(file, furi_string_get_cstr(c), len) == len;
        furi_string_free(c);
    }
    storage_file_close(file);
    storage_file_free(file);
    furi_string_free(path);
    return ok;
}

static bool de_parse_u64(const char* s, uint64_t* out) {
    uint64_t v = 0;
    for(uint8_t i = 0; i < 16; i++) {
        char c = s[i];
        uint8_t nib;
        if(c >= '0' && c <= '9')
            nib = c - '0';
        else if(c >= 'A' && c <= 'F')
            nib = c - 'A' + 10;
        else if(c >= 'a' && c <= 'f')
            nib = c - 'a' + 10;
        else
            return false;
        v = (v << 4) | nib;
    }
    *out = v;
    return true;
}

static bool de_load(DeApp* app, const char* path) {
    File* file = storage_file_alloc(app->storage);
    bool ok = false;
    if(storage_file_open(file, path, FSAM_READ, FSOM_OPEN_EXISTING)) {
        char buf[256] = {0};
        size_t n = storage_file_read(file, buf, sizeof(buf) - 1);
        buf[n] = 0;
        uint64_t f1 = 0, f2 = 0;
        unsigned segs = 1;
        char* p;
        if((p = strstr(buf, "Frame1:")) && de_parse_u64(p + 8, &f1) &&
           (p = strstr(buf, "Frame2:")) && de_parse_u64(p + 8, &f2)) {
            if((p = strstr(buf, "Segments:"))) segs = atoi(p + 9);
            app->frame1 = f1;
            app->frame2 = f2;
            app->segments = (segs == 2) ? 2 : 1;
            de_update_id_text(app);
            ok = true;
        }
    }
    storage_file_close(file);
    storage_file_free(file);
    return ok;
}

// ---- Submenu (main menu) ----------------------------------------------------

static void de_submenu_cb(void* context, uint32_t index) {
    DeApp* app = context;
    switch(index) {
    case DeMenuRead:
        de_start_read(app);
        break;
    case DeMenuSaved: {
        // pick a saved card via file browser, then show its detail
        DialogsFileBrowserOptions opts;
        dialog_file_browser_set_basic_options(&opts, DE_EXTENSION, NULL);
        opts.base_path = DE_FOLDER;
        FuriString* path = furi_string_alloc_set(DE_FOLDER);
        if(dialog_file_browser_show(app->dialogs, path, path, &opts) &&
           de_load(app, furi_string_get_cstr(path))) {
            furi_string_set(app->loaded_path, path);
            de_show_card_detail(app);
        } else {
            de_show_menu(app);
        }
        furi_string_free(path);
        break;
    }
    case DeMenuAddManual:
        app->pending = DeInputAddFrames;
        memset(app->byte_buf, 0, sizeof(app->byte_buf));
        byte_input_set_header_text(app->byte_input, "Frame1(8B)+Frame2(8B) hex");
        de_switch(app, DeViewByteInput);
        break;
    case DeMenuAbout: {
        widget_reset(app->widget);
        widget_add_text_scroll_element(
            app->widget,
            0,
            0,
            128,
            64,
            "Dual EM4100 Tool\n\nReads & emulates dual-frame\n128-bit EM4100 cards\n(MAXBLOCK=4) that standard\ntools only half-copy.\n\nAlso handles ordinary\nsingle-frame EM4100.");
        de_switch(app, DeViewWidget);
        break;
    }
    }
}

static void de_show_menu(DeApp* app) {
    app->submenu_is_detail = false;
    submenu_reset(app->submenu);
    submenu_set_header(app->submenu, "Dual EM4100 Tool");
    submenu_add_item(app->submenu, "Read card", DeMenuRead, de_submenu_cb, app);
    submenu_add_item(app->submenu, "Saved cards", DeMenuSaved, de_submenu_cb, app);
    submenu_add_item(app->submenu, "Add manually", DeMenuAddManual, de_submenu_cb, app);
    submenu_add_item(app->submenu, "About", DeMenuAbout, de_submenu_cb, app);
    de_switch(app, DeViewSubmenu);
}

// ---- Read flow --------------------------------------------------------------

static void de_read_finish(DeApp* app) {
    app->read_running = false;
    if(app->read_thread) {
        furi_thread_join(app->read_thread);
        furi_thread_free(app->read_thread);
        app->read_thread = NULL;
    }
}

// popup "Reading..." needs to poll the worker; we use a periodic timer via the
// popup callback is not enough, so we drive it from a FuriTimer.
static void de_read_timer_cb(void* context) {
    DeApp* app = context;
    if(app->read_found) {
        furi_timer_stop(app->read_timer);
        de_read_finish(app);
        notification_message(app->notifications, &sequence_success);
        notification_message(app->notifications, &sequence_single_vibro);
        de_update_id_text(app);
        de_show_read_result(app);
    } else if(app->read_thread &&
              furi_thread_get_state(app->read_thread) == FuriThreadStateStopped) {
        // worker exited without a find -> timeout
        furi_timer_stop(app->read_timer);
        de_read_finish(app);
        popup_reset(app->popup);
        popup_set_header(app->popup, "No card found", 64, 18, AlignCenter, AlignTop);
        popup_set_text(
            app->popup, "Reposition the card\nand try again.", 64, 34, AlignCenter, AlignTop);
        de_switch(app, DeViewPopup);
    }
}

static void de_start_read(DeApp* app) {
    // capture stream is (re)allocated inside de_capture_decode
    app->read_running = true;
    app->read_found = false;
    app->read_thread = furi_thread_alloc_ex("DeRead", 2048, de_read_thread, app);
    furi_thread_start(app->read_thread);

    popup_reset(app->popup);
    popup_set_header(app->popup, "Reading...", 64, 18, AlignCenter, AlignTop);
    popup_set_text(
        app->popup, "Hold card on the\nback of the Flipper", 64, 34, AlignCenter, AlignTop);
    de_switch(app, DeViewPopup);

    furi_timer_start(app->read_timer, furi_ms_to_ticks(100));
}

// Build the read-result / card-detail widget. dual frame shown in full.
static void de_fill_card_widget(DeApp* app, bool from_read) {
    widget_reset(app->widget);
    uint8_t id[5];
    de_frame_to_id(app->frame1, id);

    char line[40];
    widget_add_string_element(
        app->widget, 2, 2, AlignLeft, AlignTop, FontPrimary,
        from_read ? "Card read!" : "Saved card");

    snprintf(line, sizeof(line), "ID: %02X %02X %02X %02X %02X", id[0], id[1], id[2], id[3], id[4]);
    widget_add_string_element(app->widget, 2, 18, AlignLeft, AlignTop, FontSecondary, line);

    if(app->segments == 2) {
        widget_add_string_element(
            app->widget, 2, 30, AlignLeft, AlignTop, FontSecondary, "2nd segment:");
        uint8_t* f2 = (uint8_t*)&app->frame2;
        // frame2 is a uint64; print big-endian bytes
        snprintf(
            line, sizeof(line), "%02X %02X %02X %02X %02X %02X %02X %02X",
            (uint8_t)(app->frame2 >> 56), (uint8_t)(app->frame2 >> 48),
            (uint8_t)(app->frame2 >> 40), (uint8_t)(app->frame2 >> 32),
            (uint8_t)(app->frame2 >> 24), (uint8_t)(app->frame2 >> 16),
            (uint8_t)(app->frame2 >> 8), (uint8_t)(app->frame2));
        widget_add_string_element(app->widget, 2, 41, AlignLeft, AlignTop, FontSecondary, line);
        UNUSED(f2);
    } else {
        widget_add_string_element(
            app->widget, 2, 30, AlignLeft, AlignTop, FontSecondary, "Single-frame (64-bit)");
    }
}

// ============================================================================
// Read-result screen: buttons Save / (Back)
// ============================================================================

typedef enum {
    DeBtnSave,
    DeBtnEmulate,
    DeBtnRename,
    DeBtnDelete,
} DeButtonId;

// Distinct button callbacks keep things simple.
static void de_btn_save_cb(GuiButtonType t, InputType in, void* ctx) {
    UNUSED(t);
    DeApp* app = ctx;
    if(in != InputTypeShort) return;
    // default name = ID hex
    uint8_t id[5];
    de_frame_to_id(app->frame1, id);
    snprintf(
        app->name_buf, sizeof(app->name_buf), "%02X%02X%02X%02X%02X",
        id[0], id[1], id[2], id[3], id[4]);
    app->pending = DeInputSaveName;
    text_input_set_header_text(app->text_input, "Name the card");
    text_input_set_result_callback(
        app->text_input, de_text_input_done, app, app->name_buf, DE_NAME_LEN, false);
    de_switch(app, DeViewTextInput);
}

static void de_show_read_result(DeApp* app) {
    de_fill_card_widget(app, true);
    widget_add_button_element(
        app->widget, GuiButtonTypeCenter, "Save", de_btn_save_cb, app);
    de_switch(app, DeViewWidget);
}

// ============================================================================
// Card-detail screen: a submenu (Emulate / Write to T5577 / Rename / Delete)
// ============================================================================

typedef enum {
    DeDetailEmulate,
    DeDetailWrite,
    DeDetailRename,
    DeDetailDelete,
} DeDetailId;

static void de_detail_submenu_cb(void* context, uint32_t index) {
    DeApp* app = context;
    switch(index) {
    case DeDetailEmulate:
        de_emulate_start(app);
        popup_reset(app->popup);
        popup_set_header(app->popup, "Emulating", 64, 18, AlignCenter, AlignTop);
        popup_set_text(app->popup, app->id_text, 64, 38, AlignCenter, AlignTop);
        de_switch(app, DeViewPopup);
        break;
    case DeDetailWrite:
        de_start_write(app);
        break;
    case DeDetailRename: {
        FuriString* fname = furi_string_alloc();
        path_extract_filename(app->loaded_path, fname, true);
        strncpy(app->name_buf, furi_string_get_cstr(fname), DE_NAME_LEN - 1);
        app->name_buf[DE_NAME_LEN - 1] = 0;
        furi_string_free(fname);
        app->pending = DeInputRename;
        text_input_set_header_text(app->text_input, "Rename card");
        text_input_set_result_callback(
            app->text_input, de_text_input_done, app, app->name_buf, DE_NAME_LEN, false);
        de_switch(app, DeViewTextInput);
        break;
    }
    case DeDetailDelete:
        storage_simply_remove(app->storage, furi_string_get_cstr(app->loaded_path));
        de_show_menu(app);
        break;
    }
}

static void de_show_card_detail(DeApp* app) {
    app->submenu_is_detail = true;
    submenu_reset(app->submenu);
    submenu_set_header(app->submenu, app->id_text); // ID as header
    submenu_add_item(app->submenu, "Emulate", DeDetailEmulate, de_detail_submenu_cb, app);
    submenu_add_item(app->submenu, "Write to T5577", DeDetailWrite, de_detail_submenu_cb, app);
    submenu_add_item(app->submenu, "Rename", DeDetailRename, de_detail_submenu_cb, app);
    submenu_add_item(app->submenu, "Delete", DeDetailDelete, de_detail_submenu_cb, app);
    de_switch(app, DeViewSubmenu);
}

// ============================================================================
// Text input result (save name / rename) + byte input (manual add)
// ============================================================================

static void de_text_input_done(void* context) {
    DeApp* app = context;
    if(app->pending == DeInputSaveName) {
        de_save_as(app, app->name_buf);
        de_show_menu(app);
    } else if(app->pending == DeInputRename) {
        // rename the file, then keep the loaded_path pointing at the new name
        // and return to the card's detail menu (so the user stays on the card).
        FuriString* newp = furi_string_alloc();
        de_card_path(app, app->name_buf, newp);
        if(storage_common_rename(
               app->storage,
               furi_string_get_cstr(app->loaded_path),
               furi_string_get_cstr(newp)) == FSE_OK) {
            furi_string_set(app->loaded_path, newp);
        }
        furi_string_free(newp);
        app->pending = DeInputNone;
        de_show_card_detail(app);
        return;
    }
    app->pending = DeInputNone;
}

static void de_byte_input_done(void* context) {
    DeApp* app = context;
    // byte_buf[0..7] = frame1 big-endian, [8..15] = frame2
    uint64_t f1 = 0, f2 = 0;
    for(uint8_t i = 0; i < 8; i++) f1 = (f1 << 8) | app->byte_buf[i];
    for(uint8_t i = 0; i < 8; i++) f2 = (f2 << 8) | app->byte_buf[8 + i];
    app->frame1 = f1;
    app->frame2 = f2;
    app->segments = de_frame_valid(f2, false) ? 2 : 1;
    de_update_id_text(app);
    de_show_read_result(app); // reuse: show + Save
    app->pending = DeInputNone;
}

// ============================================================================
// ViewDispatcher navigation (back handling)
// ============================================================================

static bool de_back_cb(void* context) {
    DeApp* app = context;

    // A write in progress must not be interrupted: t5577_write() blocks inside a
    // critical section and joining it here would freeze the UI. Swallow Back and
    // let the write finish; the timer callback shows the result.
    if(app->write_running) {
        return true;
    }

    // Active operations: stop and return to menu.
    if(app->emulating) {
        de_emulate_stop(app);
        de_show_menu(app);
        return true;
    }
    if(app->read_running) {
        furi_timer_stop(app->read_timer);
        de_read_finish(app);
        de_show_menu(app);
        return true;
    }

    // Cancelling a text input: return to where the input was launched from,
    // so the user doesn't lose a just-read card or fall out of the detail menu.
    if(app->current_view == DeViewTextInput) {
        DePendingInput was = app->pending;
        app->pending = DeInputNone;
        if(was == DeInputSaveName) {
            de_show_read_result(app); // keep the read card, let them retry Save
            return true;
        }
        if(was == DeInputRename) {
            de_show_card_detail(app); // back to the card's detail menu
            return true;
        }
    }

    // Card-detail submenu -> back to main menu (not exit).
    if(app->current_view == DeViewSubmenu && app->submenu_is_detail) {
        de_show_menu(app);
        return true;
    }

    // On the main menu -> let the dispatcher exit the app.
    if(app->current_view == DeViewSubmenu) return false;

    // Any other screen (read result, about, popup) -> menu.
    app->pending = DeInputNone;
    de_show_menu(app);
    return true;
}

// ============================================================================
// Main
// ============================================================================

int32_t dual_em4100_tool_app(void* p) {
    UNUSED(p);
    DeApp* app = malloc(sizeof(DeApp));
    memset(app, 0, sizeof(DeApp));
    app->em_duration = malloc(sizeof(uint32_t) * DE_BUFFER_MAX);
    app->em_pulse = malloc(sizeof(uint32_t) * DE_BUFFER_MAX);
    app->em_stream = furi_stream_buffer_alloc(sizeof(uint32_t) * 8, sizeof(uint32_t));
    app->loaded_path = furi_string_alloc();
    app->segments = 1;

    app->gui = furi_record_open(RECORD_GUI);
    app->storage = furi_record_open(RECORD_STORAGE);
    app->dialogs = furi_record_open(RECORD_DIALOGS);
    app->notifications = furi_record_open(RECORD_NOTIFICATION);

    app->view_dispatcher = view_dispatcher_alloc();
    view_dispatcher_attach_to_gui(app->view_dispatcher, app->gui, ViewDispatcherTypeFullscreen);
    view_dispatcher_set_event_callback_context(app->view_dispatcher, app);
    view_dispatcher_set_navigation_event_callback(app->view_dispatcher, de_back_cb);

    app->submenu = submenu_alloc();
    app->popup = popup_alloc();
    app->widget = widget_alloc();
    app->text_input = text_input_alloc();
    app->byte_input = byte_input_alloc();
    app->read_timer = furi_timer_alloc(de_read_timer_cb, FuriTimerTypePeriodic, app);
    app->write_timer = furi_timer_alloc(de_write_timer_cb, FuriTimerTypePeriodic, app);

    view_dispatcher_add_view(app->view_dispatcher, DeViewSubmenu, submenu_get_view(app->submenu));
    view_dispatcher_add_view(app->view_dispatcher, DeViewPopup, popup_get_view(app->popup));
    view_dispatcher_add_view(app->view_dispatcher, DeViewWidget, widget_get_view(app->widget));
    view_dispatcher_add_view(
        app->view_dispatcher, DeViewTextInput, text_input_get_view(app->text_input));
    view_dispatcher_add_view(
        app->view_dispatcher, DeViewByteInput, byte_input_get_view(app->byte_input));

    // wire input-done callbacks (set buffers later when entering)
    text_input_set_result_callback(
        app->text_input, de_text_input_done, app, app->name_buf, DE_NAME_LEN, false);
    byte_input_set_result_callback(
        app->byte_input, de_byte_input_done, NULL, app, app->byte_buf, sizeof(app->byte_buf));

    de_show_menu(app);
    view_dispatcher_run(app->view_dispatcher);

    // Cleanup. Stop timers first (so no callback fires mid-teardown), then reap
    // worker threads, then free everything.
    furi_timer_stop(app->read_timer);
    furi_timer_stop(app->write_timer);
    de_read_finish(app);
    de_write_finish(app);
    de_emulate_stop(app);
    furi_timer_free(app->read_timer);
    furi_timer_free(app->write_timer);
    if(app->capture_stream) furi_stream_buffer_free(app->capture_stream);

    view_dispatcher_remove_view(app->view_dispatcher, DeViewSubmenu);
    view_dispatcher_remove_view(app->view_dispatcher, DeViewPopup);
    view_dispatcher_remove_view(app->view_dispatcher, DeViewWidget);
    view_dispatcher_remove_view(app->view_dispatcher, DeViewTextInput);
    view_dispatcher_remove_view(app->view_dispatcher, DeViewByteInput);
    submenu_free(app->submenu);
    popup_free(app->popup);
    widget_free(app->widget);
    text_input_free(app->text_input);
    byte_input_free(app->byte_input);
    view_dispatcher_free(app->view_dispatcher);

    furi_record_close(RECORD_NOTIFICATION);
    furi_record_close(RECORD_DIALOGS);
    furi_record_close(RECORD_STORAGE);
    furi_record_close(RECORD_GUI);

    furi_string_free(app->loaded_path);
    furi_stream_buffer_free(app->em_stream);
    free(app->em_pulse);
    free(app->em_duration);
    free(app);
    return 0;
}
