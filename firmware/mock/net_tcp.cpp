#include "net_tcp.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cstdio>
#include <cerrno>

static int  s_srv  = -1;
static int  s_conn = -1;
static bool s_just_connected = false;

static void set_nonblocking(int fd) {
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
}

namespace net_tcp {

bool accept_client() {
    if (s_conn >= 0) return true;
    socklen_t len = 0;
    int fd = accept(s_srv, nullptr, &len);
    if (fd < 0) return false;
    s_conn = fd;
    set_nonblocking(s_conn);
    s_just_connected = true;
    return true;
}

bool client_just_connected() {
    if (!s_just_connected) return false;
    s_just_connected = false;
    return true;
}

bool is_connected() { return s_conn >= 0; }

bool data_available() {
    if (s_conn < 0) return false;
    char c;
    ssize_t n = recv(s_conn, &c, 1, MSG_PEEK | MSG_DONTWAIT);
    if (n == 0) { close(s_conn); s_conn = -1; return false; }
    return n > 0;
}

int read_byte() {
    if (s_conn < 0) return -1;
    uint8_t c;
    ssize_t n = recv(s_conn, &c, 1, 0);
    if (n <= 0) {
        if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
            close(s_conn); s_conn = -1;
        }
        return -1;
    }
    return (int)c;
}

bool init(int port) {
    s_srv = socket(AF_INET, SOCK_STREAM, 0);
    if (s_srv < 0) return false;
    int yes = 1;
    setsockopt(s_srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    set_nonblocking(s_srv);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons((uint16_t)port);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    if (bind(s_srv, (sockaddr*)&addr, sizeof(addr)) < 0) return false;
    if (listen(s_srv, 1) < 0) return false;
    return true;
}

bool send(const char* data, size_t len) {
    if (s_conn < 0) return false;
    ssize_t n = ::send(s_conn, data, len, MSG_NOSIGNAL);
    if (n < 0) { close(s_conn); s_conn = -1; return false; }
    return true;
}

int readline(char* buf, int max_len) {
    if (s_conn < 0) return -1;
    int i = 0;
    while (i < max_len - 1) {
        char c;
        ssize_t n = recv(s_conn, &c, 1, 0);
        if (n == 0) { close(s_conn); s_conn = -1; return -1; }
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            close(s_conn); s_conn = -1; return -1;
        }
        buf[i++] = c;
        if (c == '\n') break;
    }
    buf[i] = '\0';
    return i;
}

void shutdown() {
    if (s_conn >= 0) { close(s_conn); s_conn = -1; }
    if (s_srv  >= 0) { close(s_srv);  s_srv  = -1; }
}

}  // namespace net_tcp
