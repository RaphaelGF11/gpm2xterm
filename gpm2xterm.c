/*
 * gpm2xterm — GPM mouse -> xterm SGR escape sequences + OSC52 clipboard
 * emulation for the Linux console.
 *
 * Based on https://github.com/RaphaelGF11/gpm2xterm (unlicensed at time of
 * writing). This rewrite is GPL-2.0-or-later (matches libgpm). Upstream
 * fixes included:
 *   - wheel events were unmapped (arrived as GPM_MOVE wdy=±1 and would be
 *     ignored / mis-mapped): now emitted as SGR buttons 64/65 (xterm scroll
 *     bit convention, what OpenTUI/Ghostty-class parsers expect — NOT 4/5,
 *     which decode as left/middle clicks in SGR)
 *   - defaultMask=GPM_HARD alone CONSUMES events from gpm's own selection
 *     handler (breaks middle-click paste + console selection); fixed by
 *     listing the event types so they fall through to the daemon (which has
 *     the CAP_SYS_ADMIN the modern kernel requires for TIOCL_SETSEL)
 *   - DECSET scanner: proper state machine handles multi-parameter sets
 *     (?1000;1002;1006h), split reads, and OSC/DCS across chunk boundaries
 *   - stdin EOF no longer tears down the child-output relay
 * New features:
 *   - OSC52 clipboard emulation: stores copies, answers queries, replies to
 *     XTGETTCAP "Ms" — apps (e.g. OpenCode v2) treat the console as a
 *     clipboard-capable terminal
 *   - Ctrl+V is answered by the wrapper with a bracketed-paste injection of
 *     the stored clipboard (many apps only ever WRITE OSC52; their read path
 *     assumes an OS clipboard or a Ghostty handshake the console lacks)
 *   - Enter translation: console sends identical 0x0D for Enter / Shift+Enter
 *     / Ctrl+Enter; the wrapper queries the kernel's live shift state
 *     (TIOCL_GETSHIFTSTATE) and maps modified Enter to ctrl+j (newline in
 *     most prompt UIs)
 *   - clears stuck DECSET modes (notably ?2004, which otherwise makes the
 *     kernel wrap ALL console pastes in bracket markers) at startup/exit
 *   - GPM2XTERM_DEBUG=1 traces tracking detection, gpm events, SGR output,
 *     OSC52 and the Enter translation
 *
 * Usage:  gpm2xterm <program> [args...]     (needs the gpm daemon)
 */
#define _XOPEN_SOURCE 600
#define _DEFAULT_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pty.h>
#include <termios.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/tiocl.h>      /* TIOCLINUX subcommands: TIOCL_GETSHIFTSTATE */
#include <sys/select.h>
#include <sys/wait.h>
#include <gpm.h>
#include <errno.h>
#include <fcntl.h>

static struct termios saved_term;
static int master_fd = -1;
static pid_t child_pid = -1;
static volatile sig_atomic_t running = 1;
static int gpm_available = 0;

/* Active mouse-tracking modes requested by the child program */
static int track_button = 0;   /* 1000: press/release + wheel */
static int track_cell   = 0;   /* 1002: drag (cell motion)    */
static int track_any    = 0;   /* 1003: motion without button */
static int track_focus  = 0;   /* 1004: focus in/out (unused) */
static int track_bracket = 0;  /* 2004: bracketed paste       */
static int dbg = 0;            /* GPM2XTERM_DEBUG=1 */

/* OSC52 clipboard emulation */
#define OSC52_MAX 262144
static char  cb_b64[OSC52_MAX];   /* stored copy, base64 as sent by app */
static size_t cb_len = 0;
static long long cb_last_set = 0;

/* Scanning state carried across read() boundaries */
#define CARRY_MAX 64
#define OSC_CAP   (1 << 20)
static char  osc_buf[OSC_CAP];     /* in-progress ESC ] ... */
static size_t osc_len = 0;
static int   osc_active = 0;       /* saw ESC ] */
static char  dcs_buf[OSC_CAP];     /* in-progress ESC P ... */
static size_t dcs_len = 0;
static int   dcs_active = 0;
static char  carry[CARRY_MAX];     /* tail of previous chunk for DECSET scan */
static size_t carry_len = 0;

/* Kernel console shift_state semantics, see TIOCL_GETSHIFTSTATE (6) in
 * <linux/tiocl.h>. Bit layout changed across kernel versions (legacy:
 * LSHFT/RSHFT/ALT/CTRL=1/2/4/8; modern keymap weights: Shift=1 AltGr=2
 * Ctrl=4 Alt=8 ShiftL=16 ShiftR=32 CtrlL=64 CtrlR=128), so for Enter
 * translation we only test state != 0 (no modifier held => 0 in ALL
 * layouts). Mouse modifier bits below use the modern weights. */
#define MODERN_SHIFT_BITS 0x31   /* Shift|ShiftL|ShiftR */
#define MODERN_ALT_BITS   0x08   /* Alt (not AltGr) */
#define MODERN_CTRL_BITS  0xC4   /* Ctrl|CtrlL|CtrlR */

static void restore_terminal(void) {
    if (gpm_available) Gpm_Close();
    /* belt & braces: leave no stuck modes even if the child died uncleanly */
    {
        static const char reset_modes[] =
            "\x1b[?2004l\x1b[?1003l\x1b[?1002l\x1b[?1000l\x1b[?1006l";
        ssize_t w = write(STDOUT_FILENO, reset_modes, sizeof(reset_modes) - 1);
        (void)w;
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &saved_term);
}

static void sig_handler(int sig) {
    if (child_pid > 0) kill(child_pid, sig);
    if (sig == SIGTERM || sig == SIGINT || sig == SIGHUP) running = 0;
}

static void sigchld_handler(int sig) {
    (void)sig;
    int status;
    if (child_pid > 0 && waitpid(child_pid, &status, WNOHANG) > 0) running = 0;
}

static void setup_raw_terminal(void) {
    struct termios raw = saved_term;
    raw.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    raw.c_oflag &= ~OPOST;
    raw.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    raw.c_cflag &= ~(CSIZE | PARENB);
    raw.c_cflag |= CS8;
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

static void update_window_size(void) {
    struct winsize ws;
    if (ioctl(STDIN_FILENO, TIOCGWINSZ, &ws) == 0) ioctl(master_fd, TIOCSWINSZ, &ws);
}

static void sigwinch_handler(int sig) { (void)sig; update_window_size(); }

/* ---------------- SGR mouse injection ---------------- */

static int sgr_button(int b) {           /* GPM button bits -> SGR base */
    if (b & GPM_B_LEFT)   return 0;
    if (b & GPM_B_MIDDLE) return 1;
    if (b & GPM_B_RIGHT)  return 2;
    return -1;
}

static int sgr_mods(unsigned char m) {
    /* map kernel shift_state (modern keymap weights) -> xterm SGR bits
     * (shift=4 alt=8 ctrl=16) */
    int r = 0;
    if (m & MODERN_SHIFT_BITS) r |= 4;
    if (m & MODERN_ALT_BITS)   r |= 8;
    if (m & MODERN_CTRL_BITS)  r |= 16;
    return r;
}

static void send_sgr(int btn, int x, int y, char action, unsigned char mods) {
    char seq[48];
    int len = snprintf(seq, sizeof(seq), "\x1b[<%d;%d;%d%c",
                       btn | sgr_mods(mods), x, y, action);
    if (len > 0) {
        if (dbg) fprintf(stderr, "[gpm2xterm] -> SGR %s\n", seq + 1);
        ssize_t r = write(master_fd, seq, (size_t)len); (void)r;
    }
}

static void forward_gpm_event(Gpm_Event *ev);
static void inject_paste(void);

static void forward_gpm_event(Gpm_Event *ev) {
    int btn;
    if (dbg)
        fprintf(stderr, "[gpm2xterm] gpm ev type=%d btn=%d x=%d y=%d wdy=%d "
                "(1000=%d 1002=%d 1003=%d)\n",
                GPM_BARE_EVENTS(ev->type), ev->buttons, ev->x, ev->y, ev->wdy,
                track_button, track_cell, track_any);
    if (!(track_button || track_cell || track_any)) return;

    /* Wheel: xterm SGR scroll-bit encoding (64=up, 65=down). Press-ONLY:
     * a trailing '\e[<64m' release re-decodes as button-0 (64&3) in strict
     * parsers (e.g. OpenTUI) and synthesizes a phantom left-CLICK at the
     * cursor — which OpenCode treats as a toggle on collapsible blocks. */
    if (ev->wdy) {
        if (track_button)
            send_sgr(ev->wdy > 0 ? 64 : 65, ev->x, ev->y, 'M', ev->modifiers);
        return;
    }

    /* NOTE: console selection/pointer/paste are handled by the gpm daemon
     * itself (root) - defaultMask passes events through to it; a non-root
     * client cannot TIOCL_SETSEL (kernel requires CAP_SYS_ADMIN). */
    switch (GPM_BARE_EVENTS(ev->type)) {
    case GPM_DOWN:
        btn = sgr_button(ev->buttons);
        if (btn >= 0) send_sgr(btn, ev->x, ev->y, 'M', ev->modifiers);
        break;
    case GPM_UP:
        btn = sgr_button(ev->buttons);
        if (btn >= 0 && track_button)
            send_sgr(btn, ev->x, ev->y, 'm', ev->modifiers);
        break;
    case GPM_DRAG:
        btn = sgr_button(ev->buttons);
        if (btn >= 0 && track_cell)
            send_sgr(btn + 32, ev->x, ev->y, 'M', ev->modifiers);
        break;
    case GPM_MOVE:
        if (track_any) send_sgr(35, ev->x, ev->y, 'M', ev->modifiers);
        break;
    }
}

/* ---------------- OSC52 / XTGETTCAP emulation ---------------- */

static void reply_osc52(const char *sel) {
    char hdr[40];
    char codes[16];
    snprintf(codes, sizeof(codes), "%.15s", (sel && *sel) ? sel : "C");
    snprintf(hdr, sizeof(hdr), "\x1b]52;%s;", codes);
    ssize_t r;
    r = write(master_fd, hdr, strlen(hdr)); (void)r;
    if (cb_len) {
        r = write(master_fd, cb_b64, cb_len); (void)r;
    }
    r = write(master_fd, "\x1b\\", 2); (void)r;
}

static long long mono_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
}

static void handle_osc52(char *p) {         /* p = payload after "52;" */
    char *codes = p;
    char *data = strchr(codes, ';');
    if (!data) return;
    *data++ = 0;
    if (strcmp(data, "?") == 0) {
        if (dbg) fprintf(stderr, "[gpm2xterm] osc52 query (sel=%s) -> reply %zu b64 bytes\n",
                         codes, cb_len);
        reply_osc52(codes);
        return;
    }
    if (dbg) fprintf(stderr, "[gpm2xterm] osc52 set (sel=%s) %zu b64 bytes\n", codes, strlen(data));
    size_t dlen = strlen(data);
    /* A fresh copy gesture starts a new accumulation; back-to-back
     * fragments (xterm-style idle timeout) extend the previous one. */
    long long now = mono_us();
    if (cb_len == 0 || (now - cb_last_set) > 250000) cb_len = 0;
    if (cb_len + dlen < OSC52_MAX) {
        memcpy(cb_b64 + cb_len, data, dlen);
        cb_len += dlen;
        cb_b64[cb_len] = 0;
    }
    cb_last_set = now;
}

/* Minimal base64 decoder for OSC52 clipboard paste. */
static int b64val(int c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}
static size_t b64decode(const char *in, size_t inlen, char *out, size_t outmax) {
    size_t o = 0; int val = 0, bits = 0;
    for (size_t i = 0; i < inlen; i++) {
        int d = b64val((unsigned char)in[i]);
        if (d < 0) continue;                 /* skip '=' padding / newlines */
        val = (val << 6) | d; bits += 6;
        if (bits >= 8) { bits -= 8; if (o < outmax) out[o++] = (char)((val >> bits) & 0xff); }
    }
    if (o < outmax) out[o] = 0;
    return o;
}

/* Emulate terminal "paste" for Ctrl+V: inject the stored OSC52 clipboard
 * into the child as (bracketed) text. OpenCode's terminal provider has no
 * read() and its native read is gated on a handshake the console lacks, so
 * it never sends an OSC52 query we could answer — we satisfy the paste
 * ourselves using the copy OpenCode already put on the clipboard. */
static void inject_paste(void) {
    if (cb_len == 0) return;
    static char text[OSC52_MAX];
    size_t n = b64decode(cb_b64, cb_len, text, sizeof(text) - 1);
    ssize_t r;
    if (track_bracket) { r = write(master_fd, "\x1b[200~", 6); (void)r; }
    r = write(master_fd, text, n); (void)r;
    if (track_bracket) { r = write(master_fd, "\x1b[201~", 6); (void)r; }
    if (dbg) fprintf(stderr, "[gpm2xterm] Ctrl+V -> injected %zu byte paste\n", n);
}

static int hexval(int c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void handle_gettcap(char *p) {
    char dec[128]; size_t n = 0;
    for (char *q = p; q[0] && q[1] && n + 1 < sizeof(dec); q += 2) {
        int hi = hexval((unsigned char)q[0]), lo = hexval((unsigned char)q[1]);
        if (hi < 0 || lo < 0) break;
        dec[n++] = (char)(hi * 16 + lo);
    }
    dec[n] = 0;
    if (dbg) fprintf(stderr, "[gpm2xterm] gettcap query: %s\n", dec);
    /* reply to every requested capability we know, ':'-joined per spec */
    char rep[256]; size_t k = 0; int matched = 0; char *save = NULL;
    for (char *tok = strtok_r(dec, ";", &save); tok; tok = strtok_r(NULL, ";", &save)) {
        if (strcmp(tok, "Ms") == 0) {
            if (matched && k + 1 < sizeof(rep)) rep[k++] = ':';
            k += (size_t)snprintf(rep + k, sizeof(rep) - k,
                                  "4d73=1b5d35323b25703125733b257032257307");
            matched = 1;
        }
        /* deliberately no "Tc": the console has no truecolor */
    }
    if (!matched) return;
    char out[300];
    int len = snprintf(out, sizeof(out), "\x1bP1r%s\x1b\\", rep);
    if (dbg) fprintf(stderr, "[gpm2xterm] gettcap: answered Ms\n");
    if (len > 0) { ssize_t w = write(master_fd, out, (size_t)len); (void)w; }
}

/* Scan child output for DECSET mouse modes and OSC52/XTGETTCAP.
 * Data is still forwarded to the real console verbatim by the caller. */
static void feed_output(const char *buf, int len) {
    /* Rolling DECSET scan over carry + current chunk */
    char scanbuf[CARRY_MAX + 4096];
    size_t sl = 0;
    if (carry_len + (size_t)len <= sizeof(scanbuf)) {
        memcpy(scanbuf, carry, carry_len); sl = carry_len;
        memcpy(scanbuf + sl, buf, (size_t)len); sl += (size_t)len;
    } else {
        memcpy(scanbuf, buf, (size_t)len); sl = (size_t)len;
    }
    for (size_t i = 0; i + 4 < sl; i++) {
        if (scanbuf[i] != 0x1b || scanbuf[i+1] != '[' || scanbuf[i+2] != '?') continue;
        size_t j = i + 3;
        int args[8], nargs = 0, term = 0;
        while (j < sl) {
            int mode = 0, got = 0;
            while (j < sl && scanbuf[j] >= '0' && scanbuf[j] <= '9') {
                mode = mode * 10 + (scanbuf[j++] - '0'); got = 1;
            }
            if (!got) break;
            if (nargs < 8) args[nargs++] = mode;
            if (j >= sl) break;
            if (scanbuf[j] == ';') { j++; continue; }
            if (scanbuf[j] == 'h' || scanbuf[j] == 'l') term = scanbuf[j];
            break;
        }
        if (!term || nargs == 0) continue;
        int on = (term == 'h');
        int old = track_button | track_cell<<1 | track_any<<2;
        for (int k = 0; k < nargs; k++) {
            switch (args[k]) {
            case 1000: track_button = on; break;
            case 1002: track_cell   = on; break;
            case 1003: track_any    = on; break;
            case 1004: track_focus  = on; break;
            case 2004: track_bracket = on; break;
            default: break;
            }
        }
        if (dbg && (track_button | track_cell<<1 | track_any<<2) != old)
            fprintf(stderr, "[gpm2xterm] mouse: 1000=%d 1002=%d 1003=%d\n",
                    track_button, track_cell, track_any);
        i = j;   /* resync after the parsed sequence */
    }
    /* keep tail for sequences split across reads */
    carry_len = sl > CARRY_MAX ? CARRY_MAX : sl;
    memcpy(carry, scanbuf + sl - carry_len, carry_len);

    /* OSC / DCS state machine over raw stream.
     * OSC:  ESC ] <payload> (BEL | ST)  -> capture "52;..." for clipboard
     * DCS:  ESC P <payload> ST          -> capture "+q..." XTGETTCAP queries
     * A bare ESC (not starting an ST) terminates the current string and is
     * reprocessed as a new introducer, so back-to-back queries don't merge. */
    for (int i = 0; i < len; i++) {
        unsigned char c = (unsigned char)buf[i];

        if (osc_active) {
            if (c == 0x07) {                                        /* BEL */
                if (osc_len < sizeof(osc_buf) - 1) osc_buf[osc_len] = 0;
                if (dbg) fprintf(stderr, "[gpm2xterm] OSC<%.*s>\n", 48, osc_buf);
                if (strncmp(osc_buf, "52;", 3) == 0) handle_osc52(osc_buf + 3);
                osc_active = 0; osc_len = 0;
                continue;
            }
            if (c == 0x1b) {
                if (i + 1 < len && buf[i+1] == '\\') {              /* ST */
                    i++;
                    if (osc_len < sizeof(osc_buf) - 1) osc_buf[osc_len] = 0;
                    if (dbg) fprintf(stderr, "[gpm2xterm] OSC<%.*s>\n", 48, osc_buf);
                    if (strncmp(osc_buf, "52;", 3) == 0) handle_osc52(osc_buf + 3);
                    osc_active = 0; osc_len = 0;
                    continue;
                }
                /* bare ESC: close current OSC, fall through to reprocess ESC */
                if (osc_len < sizeof(osc_buf) - 1) osc_buf[osc_len] = 0;
                if (dbg) fprintf(stderr, "[gpm2xterm] OSC<%.*s>\n", 48, osc_buf);
                if (strncmp(osc_buf, "52;", 3) == 0) handle_osc52(osc_buf + 3);
                osc_active = 0; osc_len = 0;
            } else {
                if (osc_len < sizeof(osc_buf) - 1) osc_buf[osc_len++] = (char)c;
                continue;
            }
        }

        if (dcs_active) {
            if (c == 0x1b) {
                if (i + 1 < len && buf[i+1] == '\\') {              /* ST */
                    i++;
                    if (dcs_len < sizeof(dcs_buf) - 1) dcs_buf[dcs_len] = 0;
                    if (dbg) fprintf(stderr, "[gpm2xterm] DCS<%.*s>\n", 48, dcs_buf);
                    if (dcs_len >= 2 && dcs_buf[0] == '+' && dcs_buf[1] == 'q')
                        handle_gettcap(dcs_buf + 2);
                    dcs_active = 0; dcs_len = 0;
                    continue;
                }
                /* bare ESC: close current DCS, fall through to reprocess ESC */
                if (dcs_len < sizeof(dcs_buf) - 1) dcs_buf[dcs_len] = 0;
                if (dbg) fprintf(stderr, "[gpm2xterm] DCS<%.*s>\n", 48, dcs_buf);
                if (dcs_len >= 2 && dcs_buf[0] == '+' && dcs_buf[1] == 'q')
                    handle_gettcap(dcs_buf + 2);
                dcs_active = 0; dcs_len = 0;
            } else {
                if (dcs_len < sizeof(dcs_buf) - 1) dcs_buf[dcs_len++] = (char)c;
                continue;
            }
        }

        if (c == 0x1b && i + 1 < len) {
            if (buf[i+1] == ']') { osc_active = 1; osc_len = 0; i += 1; }
            else if (buf[i+1] == 'P') { dcs_active = 1; dcs_len = 0; i += 1; }
        }
    }
}

/* ---------------- main ---------------- */

static int init_gpm(void) {
    Gpm_Connect conn;
    conn.eventMask   = GPM_MOVE | GPM_DRAG | GPM_DOWN | GPM_UP;
    /* GPM_HARD + all type bits: every event is delivered to us AND still
     * falls through to gpm's own (root) selection handler — the kernel now
     * requires CAP_SYS_ADMIN for TIOCL_SETSEL, so a non-root wrapper cannot
     * set the console selection itself. This gives: console pointer,
     * drag->kernel selection (paste-able on other VTs), middle-click paste.
     * (defaultMask=GPM_HARD alone would CONSUME events from the daemon —
     * that was the earlier middle-click bug.) */
    conn.defaultMask = GPM_HARD | GPM_MOVE | GPM_DRAG | GPM_DOWN | GPM_UP;
    conn.minMod = 0;
    conn.maxMod = 0;
    if (Gpm_Open(&conn, 0) < 0) return 0;
    return gpm_fd > 0 ? 1 : 0;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <program> [args...]\n", argv[0]);
        return 1;
    }
    if (getenv("GPM2XTERM_DEBUG")) dbg = 1;

    tcgetattr(STDIN_FILENO, &saved_term);
    atexit(restore_terminal);

    /* Clear modes that a crashed/killed TUI may have left stuck on this
     * console. ?2004 in particular makes the kernel wrap ALL console pastes
     * (gpm middle-click) in bracketed-paste markers, garbage-ing other VTs. */
    {
        static const char reset_modes[] =
            "\x1b[?2004l\x1b[?1003l\x1b[?1002l\x1b[?1000l\x1b[?1006l";
        ssize_t w = write(STDOUT_FILENO, reset_modes, sizeof(reset_modes) - 1);
        (void)w;
    }

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = sig_handler;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGHUP, &sa, NULL);
    sigaction(SIGQUIT, &sa, NULL);

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = sigchld_handler;
    sa.sa_flags = SA_RESTART | SA_NOCLDSTOP;
    sigaction(SIGCHLD, &sa, NULL);

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = sigwinch_handler;
    sa.sa_flags = SA_RESTART;
    sigaction(SIGWINCH, &sa, NULL);

    struct winsize ws;
    ioctl(STDIN_FILENO, TIOCGWINSZ, &ws);

    child_pid = forkpty(&master_fd, NULL, NULL, &ws);
    if (child_pid < 0) { perror("forkpty"); return 1; }
    if (child_pid == 0) {
        unsetenv("LINES"); unsetenv("COLUMNS");
        execvp(argv[1], argv + 1);
        perror("exec");
        _exit(127);
    }

    setup_raw_terminal();
    gpm_available = init_gpm();
    if (!gpm_available)
        fprintf(stderr, "gpm2xterm: GPM not available (mouse support disabled)\n");

    fd_set fds;
    char buf[4096];
    Gpm_Event ev;
    int stdin_eof = 0;
    int stdin_is_tty = isatty(STDIN_FILENO);

    while (running) {
        FD_ZERO(&fds);
        if (!stdin_eof) FD_SET(STDIN_FILENO, &fds);
        FD_SET(master_fd, &fds);
        int maxfd = master_fd;
        if (!stdin_eof && STDIN_FILENO > maxfd) maxfd = STDIN_FILENO;
        if (gpm_available && gpm_fd > 0) {
            FD_SET(gpm_fd, &fds);
            if (gpm_fd > maxfd) maxfd = gpm_fd;
        }
        maxfd++;

        int ret = select(maxfd, &fds, NULL, NULL, NULL);
        if (ret < 0) {
            if (errno == EINTR) continue;
            perror("select");
            break;
        }

        if (!stdin_eof && FD_ISSET(STDIN_FILENO, &fds)) {
            int n = (int)read(STDIN_FILENO, buf, sizeof(buf));
            if (n > 0) {
                if (n == 1 && (unsigned char)buf[0] == 0x16 && cb_len > 0) {
                    inject_paste();          /* Ctrl+V with a stored copy */
                } else if (n == 1 && (unsigned char)buf[0] == 0x0d
                           && stdin_is_tty) {
                    /* The console sends the same 0x0D for Enter, Shift+Enter
                     * and Ctrl+Enter (no CSI-u/kitty). Query the kernel's
                     * live shift state; any non-zero value means a modifier
                     * was held -> translate to ctrl+j (0x0A), which OpenCode
                     * binds to input_newline. */
                    char ss = TIOCL_GETSHIFTSTATE;
                    int ssok = (ioctl(STDIN_FILENO, TIOCLINUX, &ss) == 0);
                    if (dbg) fprintf(stderr, "[gpm2xterm] Enter, shift-state=0x%02x\n",
                                     ssok ? (unsigned char)ss : 0xFF);
                    if (ssok && (unsigned char)ss != 0) {
                        if (dbg) fprintf(stderr, "[gpm2xterm]   -> translated to ctrl+j\n");
                        char nl = 0x0a;
                        ssize_t w = write(master_fd, &nl, 1); (void)w;
                    } else {
                        ssize_t w = write(master_fd, buf, (size_t)n); (void)w;
                    }
                } else {
                    ssize_t w = write(master_fd, buf, (size_t)n); (void)w;
                }
            }
            else if (n == 0) {
                /* On a raw tty (VMIN=0) 0 means "no data", not EOF. */
                if (!stdin_is_tty) stdin_eof = 1;  /* keep relaying output */
            }
            else if (errno != EINTR) break;
        }

        if (FD_ISSET(master_fd, &fds)) {
            int n = (int)read(master_fd, buf, sizeof(buf));
            if (n > 0) {
                feed_output(buf, n);
                ssize_t w = write(STDOUT_FILENO, buf, (size_t)n);
                if (w < 0) break;
            } else if (n == 0) break;
        }

        if (gpm_available && gpm_fd > 0 && FD_ISSET(gpm_fd, &fds)) {
            /* ONE event per select iteration: Gpm_GetEvent() reads the
             * socket blocking, a drain loop would stall the whole relay. */
            if (Gpm_GetEvent(&ev) > 0) {
                Gpm_DrawPointer(ev.x, ev.y, gpm_consolefd); /* best effort */
                forward_gpm_event(&ev);
            }
        }
    }

    restore_terminal();
    if (child_pid > 0) {
        int status;
        waitpid(child_pid, &status, 0);
        if (WIFEXITED(status)) return WEXITSTATUS(status);
        if (WIFSIGNALED(status)) return 128 + WTERMSIG(status);
    }
    return 0;
}
