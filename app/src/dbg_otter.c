// Keefe Johnson
// OTTER Programmer/Debugger
/* Version 0.5 */

// Credit for initial terminal config code to:
// https://www.gnu.org/software/libc/manual/html_node/Noncanon-Example.html

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <termios.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <strings.h>
#include <readline/readline.h>
#include <readline/history.h>

/* Use this variable to remember original terminal attributes. */
struct termios saved_attributes;

volatile sig_atomic_t ctrlc = 0;

int serial_port;
uint32_t data_cksum;

#define APPLY_CMD_CKSUM(x) ((x) << 8) ^ ((x) & 0xFF) ^ (((x) & 0xFF00) >> 8) ^ (((x) & 0xFF0000) >> 16)

#define TIMEOUT_MSEC 5000

#define PARSE_ERROR_MSG "error - enter 'help' for usage"
#define EQUAL 0

// communication format:
// all communication is in 32-bit words
//
//   debugger                              <-->  PC
//   -----------------------------------------------------------------
//                                         <--   cmd (1 word)
//   echo of cmd (1 word)                   -->
//                                         <--   args+wdata (0+ words)
//   rdata (0+ words)                       -->
//   cksum of args+wdata+rdata (0/1 word)   -->
//
// if any failure occurs, the exchange is terminated early and an error code is sent to the PC, in which case the
//   action may have been partially completed/committed
// TODO: implement propagation of a signal (resulting in an error code to PC) for serial timeout mid-word
// the cksum is sent if and only if the appropriate number of words are received/sent after the cmd echo
// the cksum is currently a simple cumulative xor of all words
// TODO: replace XOR with a more robust checksum algo (CRC32?)
// TODO: prevent collision of cksum with error code(s)
// the echo of cmd is sent immediately except for the pause command, where status is confirmed before echo
// the cksum for args+wdata is sent only after the wdata has been committed to the MCU
// registers include RF, CSR, and PC, with the addr distinguishing among them:
//   0-31: RF
//   32: PC
//   65-4160: CSR (addr = CSR # + 65)
//   others: undefined behavior

#define CMD_RESET_ON        0x0FF000  // no args, no wdata, no rdata
#define CMD_RESET_OFF       0x0FF001  // no args, no wdata, no rdata
#define CMD_WRITE_MEM_RANGE 0x0FF002  // args = {addr, num_words}, wdata = 1+ words, no rdata
#define CMD_READ_MEM_RANGE  0x0FF003  // TODO: implement
#define CMD_VERSION         0x0FF004  // TODO: implement
#define CMD_PAUSE           0x0FF005  // no args, no wdata, no rdata 
#define CMD_STEP            0x0FF006  // no args, no wdata, no rdata 
#define CMD_CONTINUE        0x0FF007  // no args, no wdata, no rdata
#define CMD_STATUS          0x0FF008  // no args, no wdata, rdata = {bit1: paused, bit0: reset}
#define CMD_READ_MEM        0x0FF009  // args = {addr}, no wdata, rdata = 1 word
#define CMD_WRITE_MEM       0x0FF00A  // args = {addr}, wdata = 1 word, no rdata
#define CMD_READ_REG        0x0FF00B  // args = {addr}, no wdata, rdata = 1 word
#define CMD_WRITE_REG       0x0FF00C  // args = {addr}, wdata = 1 word, no rdata
#define CMD_SET_HW_BREAK    0x0FF00D  // args = {index}, wdata = {addr}, no rdata
#define CMD_CLR_HW_BREAK    0x0FF00E  // args = {index}, no wdata, no rdata

#define MAX_HW_BREAKPOINTS 8

// returns 1 on case-insensitive match of s to either m1 or m2, otherwise 0
int match_strs(char *s, char *m1, char *m2) {
    return strcasecmp(s, m1) == EQUAL || strcasecmp(s, m2) == EQUAL;
}

// returns 1 (and parsed number in n from s) on success (inputs non-NULL, number in uint32 range, and no trailing chars) or 0 on failure
int parse_uint32(char *s, uint32_t *n) {
    if (!s || !n) return 0;
    char *endptr;
    long long r = strtoll(s, &endptr, 0);
    if (r < 0 || r > 0xFFFFFFFF || *endptr) return 0;
    *n = (uint32_t)r;
    return 1;
}

void exit_handler(void) {
    fprintf(stderr, "Restoring serial port settings... ");
    tcsetattr(serial_port, TCSANOW, &saved_attributes);
    fprintf(stderr, "closing port... ");
    close(serial_port);
    fprintf(stderr, "closed\n");
}

void open_serial(char *path) {
    struct termios tattr;

    fprintf(stderr, "Opening serial port... ");

    if ((serial_port = open(path, O_RDWR)) == -1) {
        fprintf(stderr, "open(%s): %s\n", path, strerror(errno));
        exit(EXIT_FAILURE);
    }

    /* Make sure the port is a terminal. */
    if (!isatty(serial_port)) {
        fprintf(stderr, "Not a terminal.\n");
        exit(EXIT_FAILURE);
    }

    /* Save the terminal attributes so we can restore them later. */
    fprintf(stderr, "reading old settings... ");
    tcgetattr(serial_port, &saved_attributes);
    atexit(exit_handler);

    /* Set the funny terminal modes. */
    tcgetattr(serial_port, &tattr);
    tattr.c_oflag &= ~OPOST;  // raw output
    tattr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | ECHONL | IEXTEN);  // raw input
    tattr.c_cflag &= ~(CSIZE | PARENB | CSTOPB);  // 8N1 ...
    tattr.c_cflag |= (CS8 | CLOCAL | CREAD);      // ... and enable without ownership
    tattr.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);  // more raw input, and no software flow control
    tattr.c_cc[VMIN] = 4;
    tattr.c_cc[VTIME] = 10;  // allow up to 1.0 secs between bytes received
    cfsetospeed(&tattr, B115200);
    fprintf(stderr, "flushing transmit buffer and setting raw mode... ");
    tcsetattr(serial_port, TCSAFLUSH, &tattr);

    fprintf(stderr, "ready to communicate\n");
}

int open_file(char *path, off_t *num_words) {
    int file;
    struct stat s;

    if ((file = open(path, O_RDONLY)) == -1) {
        fprintf(stderr, "open(%s): %s\n", path, strerror(errno));
        exit(EXIT_FAILURE);
    }
    if (fstat(file, &s) == -1) {
        perror("fstat(file)");
        exit(EXIT_FAILURE);
    }

    *num_words = (s.st_size + 3) / 4;  // round up to nearest 4 bytes
    return file;
}

uint32_t file_read_word(int file) {
    uint32_t w;
    ssize_t br;
    w = 0;
    br = read(file, &w, 4);
    if (ctrlc) exit(EXIT_FAILURE);
    if (br == -1) {
        perror("read(file)");
        exit(EXIT_FAILURE);
    }
    if (br == 0) {
        fprintf(stderr, "File size changed\n");
        exit(EXIT_FAILURE);
    }
    return w;
}

void send_word(uint32_t w) {
    ssize_t bw;
    w = htonl(w);
    bw = write(serial_port, &w, 4);
    if (ctrlc) exit(EXIT_FAILURE);
    if (bw == -1) {
        perror("write(serial)");
        exit(EXIT_FAILURE);
    }
    if (bw != 4) {
        fprintf(stderr, "Wrote only %ld of 4 bytes\n", bw);
        exit(EXIT_FAILURE);
    }
}

uint32_t recv_word(void) {
    uint32_t w;
    ssize_t br;
    w = 0;
    br = read(serial_port, &w, 4);
    if (ctrlc) exit(EXIT_FAILURE);
    if (br == -1) {
        perror("read(serial)");
        exit(EXIT_FAILURE);
    }
    if (br != 4) {
        fprintf(stderr, "Read only %ld of 4 bytes: 0x%08X\n", br, ntohl(w));
        exit(EXIT_FAILURE);
    }
    return ntohl(w);
}

int wait_readable(int msec) {
    int r;
    fd_set set;
    struct timeval timeout;
    FD_ZERO(&set);
    FD_SET(serial_port, &set);
    timeout.tv_sec = msec / 1000;
    timeout.tv_usec = (msec % 1000) * 1000;
    r = select(serial_port + 1, &set, NULL, NULL, &timeout);
    if (ctrlc) exit(EXIT_FAILURE);
    if (r == -1) {
        perror("select");
        exit(EXIT_FAILURE);
    }
    return FD_ISSET(serial_port, &set);
}

void expect_word(uint32_t expect) {
    uint32_t r;
    if (wait_readable(TIMEOUT_MSEC)) {
        if ((r = recv_word()) != expect) {
            fprintf(stderr, "Expected 0x%08X but received 0x%08X\n", expect, r);
            exit(EXIT_FAILURE);
        }
    } else {
        fprintf(stderr, "Expected 0x%08X but received nothing\n", expect);
        exit(EXIT_FAILURE);
    }
}

uint32_t expect_any_word() {
    if (wait_readable(TIMEOUT_MSEC)) {
        return recv_word();
    } else {
        fprintf(stderr, "Expected a word but received nothing\n");
        exit(EXIT_FAILURE);
    }
}

void expect_timeout(void) {
    if (wait_readable(TIMEOUT_MSEC)) {
        fprintf(stderr, "Expected timeout but received 0x%08X\n", recv_word());
        exit(EXIT_FAILURE);
    }
}

void enter_reset_state(void) {
    uint32_t cmd;
    cmd = APPLY_CMD_CKSUM(CMD_RESET_ON);  // rst <= 1
    fprintf(stderr, "Putting MCU into reset state... ");
    send_word(cmd);
    expect_word(cmd);  // expect cmd echo
    fprintf(stderr, "success\n");
}

void exit_reset_state(void) {
    uint32_t cmd;
    cmd = APPLY_CMD_CKSUM(CMD_RESET_OFF);  // rst <= 0
    fprintf(stderr, "Taking MCU out of reset state... ");
    send_word(cmd);
    expect_word(cmd);  // expect cmd echo
    fprintf(stderr, "success\n");
}

void start_write_mem_range(uint32_t start_addr, uint32_t num_words) {
    uint32_t cmd;
    cmd = APPLY_CMD_CKSUM(CMD_WRITE_MEM_RANGE);
    fprintf(stderr, "Starting mem write... ");
    send_word(cmd);
    expect_word(cmd);  // expect cmd echo
    fprintf(stderr, "sending address and length... ");
    data_cksum = 0;
    send_word(start_addr);
    data_cksum ^= start_addr;
    send_word(num_words);
    data_cksum ^= num_words;
    fprintf(stderr, "sending data... ");
}

void write_mem_range_word(uint32_t word) {
    send_word(word);
    data_cksum ^= word;
}

void verify_checksum(void) {
    fsync(serial_port);
    fprintf(stderr, "verifying checksum... ");
    expect_word(data_cksum);
    fprintf(stderr, "success\n");
}

void flush_and_show_progress(long long complete, long long total) {
    static long long old_progress;
    long long progress;

    if (complete == -1) {
        fprintf(stderr, "             ");
    } else {
        progress = complete * 100 / total;
        if (progress != old_progress) {
            fsync(serial_port);
            fprintf(stderr, "\b\b\b\b\b\b\b\b\b\b\b\b\b%3lld%% done... ", progress);
            old_progress = progress;
        }
    }
}

void write_mem_range(uint32_t start_addr, char *filepath) {
    off_t num_words;
    int file;
    off_t i;
    uint32_t word;

    file = open_file(filepath, &num_words);
    fprintf(stderr, "File length is %ld words\n", num_words);

    start_write_mem_range(start_addr, num_words);
    flush_and_show_progress(-1, -1);

    for (i = 0; i < num_words; i++) {
        word = file_read_word(file);
        write_mem_range_word(word);
        flush_and_show_progress(i + 1, num_words);
    }

    verify_checksum();
    fprintf(stderr, "Successfully programmed!\n");
    close(file);
}

void set_paused_status() {
    uint32_t cmd;
    cmd = APPLY_CMD_CKSUM(CMD_PAUSE);
    fprintf(stderr, "Setting MCU paused status... ");
    send_word(cmd);
    expect_word(cmd);  // expect cmd echo
    fprintf(stderr, "success\n");
}

void clear_paused_status() {
    uint32_t cmd;
    cmd = APPLY_CMD_CKSUM(CMD_CONTINUE);
    fprintf(stderr, "Clearing MCU paused status... ");
    send_word(cmd);
    expect_word(cmd);  // expect cmd echo
    fprintf(stderr, "success\n");
}

void single_step() {
    uint32_t cmd;
    cmd = APPLY_CMD_CKSUM(CMD_STEP);
    fprintf(stderr, "Single-stepping MCU... ");
    send_word(cmd);
    expect_word(cmd);  // expect cmd echo
    fprintf(stderr, "success\n");
}

uint32_t get_status() {
    uint32_t cmd;
    cmd = APPLY_CMD_CKSUM(CMD_STATUS);
    fprintf(stderr, "Starting status request... ");
    send_word(cmd);
    expect_word(cmd);  // expect cmd echo
    data_cksum = 0;
    fsync(serial_port);
    fprintf(stderr, "receiving data... ");
    uint32_t word = expect_any_word();
    data_cksum ^= word;
    verify_checksum();
    return word;
}

void set_hw_breakpoint(uint32_t idx, uint32_t addr) {
    uint32_t cmd;
    cmd = APPLY_CMD_CKSUM(CMD_SET_HW_BREAK);
    fprintf(stderr, "Starting breakpoint set... ");
    send_word(cmd);
    expect_word(cmd);  // expect cmd echo
    fprintf(stderr, "sending index... ");
    data_cksum = 0;
    send_word(idx);
    data_cksum ^= idx;
    fprintf(stderr, "sending address... ");
    send_word(addr);
    data_cksum ^= addr;
    verify_checksum();
}

void clear_hw_breakpoint(uint32_t idx) {
    uint32_t cmd;
    cmd = APPLY_CMD_CKSUM(CMD_CLR_HW_BREAK);
    fprintf(stderr, "Starting breakpoint clear... ");
    send_word(cmd);
    expect_word(cmd);  // expect cmd echo
    fprintf(stderr, "sending index... ");
    data_cksum = 0;
    send_word(idx);
    data_cksum ^= idx;
    verify_checksum();
}

uint32_t read_mem(uint32_t addr) {
    uint32_t cmd;
    cmd = APPLY_CMD_CKSUM(CMD_READ_MEM);
    fprintf(stderr, "Starting mem read... ");
    send_word(cmd);
    expect_word(cmd);  // expect cmd echo
    fprintf(stderr, "sending address... ");
    data_cksum = 0;
    send_word(addr);
    data_cksum ^= addr;
    fsync(serial_port);
    fprintf(stderr, "receiving data... ");
    uint32_t word = expect_any_word();
    data_cksum ^= word;
    verify_checksum();
    return word;
}

void write_mem(uint32_t addr, uint32_t word) {
    uint32_t cmd;
    cmd = APPLY_CMD_CKSUM(CMD_WRITE_MEM);
    fprintf(stderr, "Starting mem write... ");
    send_word(cmd);
    expect_word(cmd);  // expect cmd echo
    fprintf(stderr, "sending address... ");
    data_cksum = 0;
    send_word(addr);
    data_cksum ^= addr;
    fprintf(stderr, "sending data... ");
    send_word(word);
    data_cksum ^= word;
    verify_checksum();
}

uint32_t read_reg(uint32_t reg) {
    uint32_t cmd;
    cmd = APPLY_CMD_CKSUM(CMD_READ_REG);
    fprintf(stderr, "Starting reg read... ");
    send_word(cmd);
    expect_word(cmd);  // expect cmd echo
    fprintf(stderr, "sending address... ");
    data_cksum = 0;
    send_word(reg);
    data_cksum ^= reg;
    fsync(serial_port);
    fprintf(stderr, "receiving data... ");
    uint32_t word = expect_any_word();
    data_cksum ^= word;
    verify_checksum();
    return word;
}

void write_reg(uint32_t reg, uint32_t word) {
    uint32_t cmd;
    cmd = APPLY_CMD_CKSUM(CMD_WRITE_REG);
    fprintf(stderr, "Starting reg write... ");
    send_word(cmd);
    expect_word(cmd);  // expect cmd echo
    fprintf(stderr, "sending address... ");
    data_cksum = 0;
    send_word(reg);
    data_cksum ^= reg;
    fprintf(stderr, "sending data... ");
    send_word(word);
    data_cksum ^= word;
    verify_checksum();
}

void ctrlc_handler(int s) {
    ctrlc = 1;
}

void register_ctrlc_handler(void) {
    struct sigaction sa = {0};
    sa.sa_handler = ctrlc_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    if (sigaction(SIGINT, &sa, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
}

int main(int argc, char *argv[]) {
    int debug;

    register_ctrlc_handler();

    if (argc == 3) {
        debug = 0;
    } else if (argc == 4 && strcmp(argv[3], "DEBUG") == 0) {
        debug = 1;
    } else {
        fprintf(stderr, "Usage: %s <mem.bin> <serial> [DEBUG]\n", argv[0]); 
        exit(EXIT_FAILURE);
    }

    open_serial(argv[2]);
    enter_reset_state();
    write_mem_range(0, argv[1]);
    if (debug) set_paused_status();
    exit_reset_state();

    while (debug) {
        char *line = readline("> ");
        if (!line) exit(EXIT_FAILURE);
        if (*line) {
            add_history(line);
            char *cmd = strtok(line, " ");
            if (match_strs(cmd, "help", "h")) {
                if (strtok(NULL, " ")) {
                    puts(PARSE_ERROR_MSG);
                } else {
                    puts("Commands:");
                    puts("  (h)elp: usage");
                    puts("  (r)eset: put OTTER into reset state");
                    puts("  (u)nreset: release OTTER from reset state");
                    puts("  (wf)ile START_ADDRESS FILE_PATH: write multiple words from a file");
                    puts("  (p)ause: pause OTTER at beginning of next instruction");
                    puts("  (s)tep: execute one instruction and pause again (same as pause if not already paused)");
                    puts("  (c)ontinue: allow OTTER to continuously execute");
                    puts("  (st)atus: report paused and reset status of OTTER");
                    puts("  (rm)em ADDRESS: read one word from OTTER memory");
                    puts("  (wm)em ADDRESS WORD: write one word to OTTER memory");
                    puts("  (rr)eg REGISTER_NUM: read one OTTER register");
                    puts("  (wr)eg REGISTER_NUM WORD: write one OTTER register");
                    puts("  (bs)et BREAKPOINT_NUM ADDRESS: set one of the hardware breakpoints to pause when program counter matches address");
                    puts("  (bc)lear BREAKPOINT_NUM: clear one of the hardware breakpoints");
                    puts("  (q)uit: exit debugger without changing OTTER state");
                    puts("Warning: Use read/write commands only while the OTTER is paused or in reset, otherwise unexpected behavior may result.");
                    puts("Pause and reset states are not mutually exclusive, and can be asserted/deasserted in any combination/order.");
                    puts("Hardware breakpoints are numbered 0-7, so no more than 8 breakpoints may be simultaneously active.");
                    puts("No notification is given when a hardware breakpoint is triggered. The status command must be used periodically to check.");
                    puts("Addresses, register numbers, and words are 32-bit unsigned and can be given in decimal, octal (0 prefix), or hex (0x prefix).");
                    puts("Registers (numbers in decimal):");
                    puts("  0-31: general purpose register file (e.g. 15 for x15/a5)");
                    puts("  32: program counter (pc)");
                    puts("  65-4160: control/status registers (add 65 to register number, e.g. 833 for CSR 768 (mstatus))");
                    puts("  others: undefined behavior");
                }
            } else if (match_strs(cmd, "reset", "r")) {
                if (strtok(NULL, " ")) {
                    puts(PARSE_ERROR_MSG);
                } else {
                    enter_reset_state();
                }
            } else if (match_strs(cmd, "unreset", "u")) {
                if (strtok(NULL, " ")) {
                    puts(PARSE_ERROR_MSG);
                } else {
                    exit_reset_state();
                }
            } else if (match_strs(cmd, "wfile", "wf")) {
                uint32_t addr;
                char *filepath;
                if (!(parse_uint32(strtok(NULL, " "), &addr))
                    || !(filepath = strtok(NULL, ""))) {  // empty delimiter string to capture remainder of line
                    puts(PARSE_ERROR_MSG);
                } else {
                    write_mem_range(addr, filepath);
                }
            } else if (match_strs(cmd, "pause", "p")) {
                if (strtok(NULL, " ")) {
                    puts(PARSE_ERROR_MSG);
                } else {
                    set_paused_status();
                }
            } else if (match_strs(cmd, "step", "s")) {
                if (strtok(NULL, " ")) {
                    puts(PARSE_ERROR_MSG);
                } else {
                    single_step();
                }
            } else if (match_strs(cmd, "continue", "c")) {
                if (strtok(NULL, " ")) {
                    puts(PARSE_ERROR_MSG);
                } else {
                    clear_paused_status();
                }
            } else if (match_strs(cmd, "status", "st")) {
                if (strtok(NULL, " ")) {
                    puts(PARSE_ERROR_MSG);
                } else {
                    switch (get_status() & 0b11) {
                        case 0b00: puts("running"); break;
                        case 0b01: puts("reset"); break;
                        case 0b10: puts("paused"); break;
                        case 0b11: puts("reset, pause pending"); break;
                    }
                }
            } else if (match_strs(cmd, "rmem", "rm")) {
                uint32_t addr;
                if (!(parse_uint32(strtok(NULL, " "), &addr))
                    || strtok(NULL, " ")) {
                    puts(PARSE_ERROR_MSG);
                } else {
                    printf("0x%08X\n", read_mem(addr));
                }
            } else if (match_strs(cmd, "wmem", "wm")) {
                uint32_t addr, word;
                if (!(parse_uint32(strtok(NULL, " "), &addr))
                    || !(parse_uint32(strtok(NULL, " "), &word))
                    || strtok(NULL, " ")) {
                    puts(PARSE_ERROR_MSG);
                } else {
                    write_mem(addr, word);
                }
            } else if (match_strs(cmd, "rreg", "rr")) {
                uint32_t reg;
                if (!(parse_uint32(strtok(NULL, " "), &reg))
                    || strtok(NULL, " ")) {
                    puts(PARSE_ERROR_MSG);
                } else {
                    printf("0x%08X\n", read_reg(reg));
                }
            } else if (match_strs(cmd, "wreg", "wr")) {
                uint32_t reg, word;
                if (!(parse_uint32(strtok(NULL, " "), &reg))
                    || !(parse_uint32(strtok(NULL, " "), &word))
                    || strtok(NULL, " ")) {
                    puts(PARSE_ERROR_MSG);
                } else {
                    write_reg(reg, word);
                }
            } else if (match_strs(cmd, "bset", "bs")) {
                uint32_t idx, addr;
                if (!(parse_uint32(strtok(NULL, " "), &idx)) || idx >= MAX_HW_BREAKPOINTS
                    || !(parse_uint32(strtok(NULL, " "), &addr))
                    || strtok(NULL, " ")) {
                    puts(PARSE_ERROR_MSG);
                } else {
                    set_hw_breakpoint(idx, addr);
                }
            } else if (match_strs(cmd, "bclear", "bc")) {
                uint32_t idx;
                if (!(parse_uint32(strtok(NULL, " "), &idx)) || idx >= MAX_HW_BREAKPOINTS
                    || strtok(NULL, " ")) {
                    puts(PARSE_ERROR_MSG);
                } else {
                    clear_hw_breakpoint(idx);
                }
            } else if (match_strs(cmd, "quit", "q")) {
                if (strtok(NULL, " ")) {
                    puts(PARSE_ERROR_MSG);
                } else {
                    debug = 0;
                }
            } else {
                puts(PARSE_ERROR_MSG);
            }
        }
        free(line);
    }

    // serial port will be closed by exit handler
}
