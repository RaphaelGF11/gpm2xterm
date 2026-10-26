/* Emulates an app like OpenCode v2: raw stdin, DECSET mouse+bracketed-paste,
 * copy via OSC52, then read stdin expecting the wrapper's Ctrl+V paste
 * injection. Run:  (sleep 0.5; printf '\026'; sleep 1) | ./gpm2xterm ./fakeapp
 * Assert: resp.bin == "\e[200~hello world\e[201~"  */
#define _DEFAULT_SOURCE
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>

#define RESP_PATH "resp.bin"

int main(void) {
    struct termios t;
    tcgetattr(0, &t);
    t.c_lflag &= ~(ICANON | ECHO | ISIG);
    t.c_iflag &= ~(IXON | ICRNL | INLCR);
    t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &t);

    fputs("\x1b[?1000;1002;1006;2004h", stdout);          /* mouse + paste */
    fputs("\x1b]52;c;aGVsbG8gd29ybGQ=\x07", stdout);       /* copy "hello world" */
    fputs("\x1b]52;c;?\x07", stdout);                      /* (also queries; wrapper replies) */
    fflush(stdout);

    FILE *out = fopen(RESP_PATH, "wb");
    fd_set f; struct timeval tv = {2, 0}; unsigned char b;
    for (int i = 0; i < 60; i++) {
        FD_ZERO(&f); FD_SET(0, &f);
        if (select(1, &f, NULL, NULL, &tv) > 0)
            if (read(0, &b, 1) == 1) fputc(b, out);
    }
    fclose(out);
    fputs("\x1b[?2004l\x1b[?1006l\x1b[?1002l\x1b[?1000l", stdout);
    return 0;
}
