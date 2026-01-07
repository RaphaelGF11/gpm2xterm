#define _XOPEN_SOURCE 600
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pty.h>
#include <termios.h>
#include <signal.h>
#include <string.h>
#include <sys/ioctl.h>
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
static int mouse_tracking_enabled = 0;

void restore_terminal() {
    tcsetattr(STDIN_FILENO, TCSANOW, &saved_term);
}

void sig_handler(int sig) {
    // Transmettre le signal à l'enfant au lieu de le tuer brutalement
    if (child_pid > 0) {
        kill(child_pid, sig);
    }
    if (sig == SIGTERM || sig == SIGINT) {
        running = 0;
    }
}

void sigchld_handler(int sig) {
    // L'enfant s'est terminé
    int status;
    if (waitpid(child_pid, &status, WNOHANG) > 0) {
        running = 0;
    }
}

void setup_raw_terminal() {
    struct termios raw = saved_term;
    
    // Mode raw complet pour capturer toutes les séquences
    raw.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    raw.c_oflag &= ~OPOST;
    raw.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    raw.c_cflag &= ~(CSIZE | PARENB);
    raw.c_cflag |= CS8;
    
    // Lecture immédiate
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

void send_xterm_mouse_sgr(struct Gpm_Event *ev) {
    char seq[32];
    int btn = 0;
    char action = 'M'; // M = press, m = release
    
    // Déterminer le bouton
    if (ev->buttons & GPM_B_LEFT)   btn = 0;
    else if (ev->buttons & GPM_B_MIDDLE) btn = 1;
    else if (ev->buttons & GPM_B_RIGHT)  btn = 2;
    
    // Type d'événement
    if (ev->type & GPM_UP) {
        action = 'm';
    } else if (ev->type & GPM_DRAG) {
        btn += 32; // Motion flag
    } else if (ev->type & GPM_MOVE) {
        btn = 35; // Move without button
    }
    
    // Format SGR: ESC[<btn;x;y M/m
    int len = snprintf(seq, sizeof(seq), "\x1b[<%d;%d;%d%c", btn, ev->x, ev->y, action);
    write(master_fd, seq, len);
}

void update_window_size(int master_fd) {
    struct winsize ws;
    if (ioctl(STDIN_FILENO, TIOCGWINSZ, &ws) == 0) {
        ioctl(master_fd, TIOCSWINSZ, &ws);
    }
}

void sigwinch_handler(int sig) {
    // Mettre à jour la taille de la fenêtre
    if (master_fd != -1) {
        update_window_size(master_fd);
    }
}

// Détecter si le programme demande mouse tracking
void check_mouse_tracking(const char *buf, int len) {
    for (int i = 0; i < len - 5; i++) {
        if (buf[i] == '\x1b' && buf[i+1] == '[' && buf[i+2] == '?') {
            // Chercher les codes de mouse tracking
            if (strstr(buf + i, "1000h") || strstr(buf + i, "1002h") || 
                strstr(buf + i, "1003h") || strstr(buf + i, "1006h")) {
                mouse_tracking_enabled = 1;
            }
            if (strstr(buf + i, "1000l") || strstr(buf + i, "1002l") || 
                strstr(buf + i, "1003l") || strstr(buf + i, "1006l")) {
                mouse_tracking_enabled = 0;
            }
        }
    }
}

int gpm_draw_pointer(Gpm_Event *ev) {
    // Utiliser la fonction native de GPM pour dessiner le curseur
    // Cela évite d'interférer avec l'affichage du programme
    return GPM_DRAWPOINTER(ev);
}

int init_gpm() {
    Gpm_Connect conn;
    
    // Demander TOUS les événements
    conn.eventMask = GPM_MOVE | GPM_DRAG | GPM_DOWN | GPM_UP;
    // GPM_HARD garde le curseur GPM visible
    conn.defaultMask = GPM_HARD;
    conn.minMod = 0;
    conn.maxMod = 0;
    
    // Essayer de se connecter à GPM en mode "raw repeater"
    // Cela permet à GPM de gérer lui-même le curseur
    int result = Gpm_Open(&conn, 0);
    
    if (result < 0) {
        return 0;
    }
    
    if (gpm_fd > 0) {
        int flags = fcntl(gpm_fd, F_GETFL);
        if (flags != -1) {
            return 1;
        }
    }
    
    return 0;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <program> [args...]\n", argv[0]);
        return 1;
    }
    
    // Sauvegarder les paramètres du terminal
    tcgetattr(STDIN_FILENO, &saved_term);
    atexit(restore_terminal);
    
    // Configuration des signaux
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = sig_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGHUP, &sa, NULL);
    sigaction(SIGQUIT, &sa, NULL);
    
    // Handler pour SIGCHLD
    sa.sa_handler = sigchld_handler;
    sa.sa_flags = SA_RESTART | SA_NOCLDSTOP;
    sigaction(SIGCHLD, &sa, NULL);
    
    // Handler pour SIGWINCH
    sa.sa_handler = sigwinch_handler;
    sigaction(SIGWINCH, &sa, NULL);
    
    // Obtenir la taille de la fenêtre
    struct winsize ws;
    ioctl(STDIN_FILENO, TIOCGWINSZ, &ws);
    
    // Créer le pseudo-terminal
    child_pid = forkpty(&master_fd, NULL, NULL, &ws);
    
    if (child_pid < 0) {
        perror("forkpty");
        return 1;
    }
    
    if (child_pid == 0) {
        // Processus enfant
        execvp(argv[1], argv + 1);
        perror("exec");
        exit(1);
    }
    
    // Processus parent
    setup_raw_terminal();
    
    // Initialiser GPM AVANT de passer en raw
    gpm_available = init_gpm();
    
    if (!gpm_available) {
        fprintf(stderr, "Warning: GPM not available (mouse support disabled)\n");
    }
    
    fd_set fds;
    char buf[4096];
    struct Gpm_Event ev;
    
    while (running) {
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        FD_SET(master_fd, &fds);
        
        int maxfd = master_fd;
        if (STDIN_FILENO > maxfd) maxfd = STDIN_FILENO;
        
        if (gpm_available && gpm_fd > 0) {
            FD_SET(gpm_fd, &fds);
            if (gpm_fd > maxfd) maxfd = gpm_fd;
        }
        
        maxfd++;
        
        int ret = select(maxfd, &fds, NULL, NULL, NULL);
        
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            perror("select");
            break;
        }
        
        // Clavier → programme
        if (FD_ISSET(STDIN_FILENO, &fds)) {
            int n = read(STDIN_FILENO, buf, sizeof(buf));
            if (n > 0) {
                write(master_fd, buf, n);
            } else if (n < 0 && errno != EINTR) {
                break;
            }
        }
        
        // Sortie programme → terminal
        if (FD_ISSET(master_fd, &fds)) {
            int n = read(master_fd, buf, sizeof(buf));
            if (n > 0) {
                // Vérifier si le programme active mouse tracking
                check_mouse_tracking(buf, n);
                write(STDOUT_FILENO, buf, n);
            } else if (n <= 0) {
                break;
            }
        }
        
        // Souris GPM → injection
        if (gpm_available && gpm_fd > 0 && FD_ISSET(gpm_fd, &fds)) {
            int result = Gpm_GetEvent(&ev);
            if (result > 0) {
                // Laisser GPM dessiner son propre curseur
                gpm_draw_pointer(&ev);
                
                // Envoyer l'événement au programme si mouse tracking activé
                if (mouse_tracking_enabled) {
                    send_xterm_mouse_sgr(&ev);
                }
            }
        }
    }
    
    // Nettoyage
    if (gpm_available) {
        Gpm_Close();
    }
    restore_terminal();
    
    // Attendre que l'enfant se termine
    if (child_pid > 0) {
        int status;
        waitpid(child_pid, &status, 0);
        if (WIFEXITED(status)) {
            return WEXITSTATUS(status);
        }
    }
    
    return 0;
}
