#ifdef _WIN32
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0501
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#define MSG_NOSIGNAL 0
#define net_errno WSAGetLastError()
#define ERR_WOULDBLOCK WSAEWOULDBLOCK
#define ERR_TRYAGAIN WSAEALREADY
#define ERR_INPROGRESS WSAEINPROGRESS
#else
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#define SOCKET int
#define INVALID_SOCKET (-1)
#define closesocket close
#define net_errno errno
#define ERR_WOULDBLOCK EWOULDBLOCK
#define ERR_TRYAGAIN EAGAIN
#define ERR_INPROGRESS EINPROGRESS
#endif

#include <SDL2/SDL_endian.h>
#include "common.h"
#include "net.h"
#include "net_internal.h"
#include "client/client.h"
#include "client/console.h"
#include "vid/vid.h"
#include <setjmp.h>
#include <uchar.h>
#include "packets.h"

#define PACKET(id, name, stuff) [id] = #name,
const char *packet_names[256] = {
#include "packets_def.h"
};
#undef PACKET

static bool init_ok = false;
static bool should_disconnect = false;

static bool read_ok = false;
static size_t total_read = 0;
static byte read_buffer[32 * 1024] = {0};
jmp_buf read_abort;

static SOCKET sockfd = INVALID_SOCKET;

#ifdef _WIN32
#define perror(desc) con_printf("%s: %d\n", desc, net_errno)
#else
#define perror(desc) con_printf("%s: %s\n", desc, strerror(net_errno))
#endif

static void setblocking(SOCKET fd, bool blocking)
{
#ifdef _WIN32
    u_long mode = !blocking;
    if(ioctlsocket(fd, FIONBIO, &mode) != 0) {
        perror("ioctlsocket");
    }
#else
    int flags;

    /* get old flags */
    if((flags = fcntl(fd, F_GETFL)) < 0) {
        perror("fcntl1");
        return;
    }

    /* add or remove nonblocking flag */
    flags = blocking ? (flags & ~O_NONBLOCK) : (flags | O_NONBLOCK);

    /* apply new flags */
    if(fcntl(fd, F_SETFL, flags) < 0) {
        perror("fcntl2");
        return;
    }
#endif
}

void disconnect_f(void);
void connect_f(void);
void say_f(void);
void respawn_f(void);

errcode net_init(void)
{
    if(init_ok)
        return ERR_OK;

#ifdef _WIN32
    WSADATA d;
    if(WSAStartup(MAKEWORD(1,1), &d) != 0) {
        perror("WSAStartup");
        return ERR_NETWORK;
    }
#endif

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if(sockfd == INVALID_SOCKET) {
        perror("socket");
        net_shutdown();
        return ERR_NETWORK;
    }

    setblocking(sockfd, false);

    init_ok = true;

    return ERR_OK;
}

void net_connect(struct sockaddr_in *sockaddr, int port)
{
    struct sockaddr_in addr = {0};
    if(sockaddr != NULL) {
        addr = *sockaddr;
        addr.sin_port = htons(port);
    } else {
        // disconnect
        addr.sin_family = AF_UNSPEC;
    }

    if(!init_ok)
        net_init();

    cl.state = cl_disconnected;

    if(connect(sockfd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        if(net_errno != ERR_TRYAGAIN && net_errno != ERR_WOULDBLOCK && net_errno != ERR_INPROGRESS) {
            perror("connect");
            net_shutdown(); // deinit
            return;
        }
    }

    if(sockaddr != NULL) {
        cl.state = cl_connecting;
    }
}

static bool net_read_packet(void);

void net_process(void)
{
    // do not do anything if not properly initialized
    if(!init_ok)
        return;

    // read and handle incoming packets
    if(cl.state > cl_disconnected) {
        if(setjmp(read_abort)) {
            // longjmp to here means that recv failed, which usually
            // means we did not receive enough data to read an entire packet

            net_write_packets();
            return;
        }

        while(net_read_packet());

        net_write_packets();
    }

    // disconnect if requested
    if(should_disconnect) {
        cl.state = cl_disconnected;
        setblocking(sockfd, true);

        // reset handshake flag and send disconnect packet
        net_write_packets();

        // wait a bit before closing the socket, didn't find a way to wait
        // for the writes to finish :/
        // fixme!
        // SDL_Delay(250);

        net_connect(NULL, 0);
        should_disconnect = false;
    }

}

// TODO
void read_entity_metadata(void)
{
    byte field_id;
    while((field_id = net_read_byte()) != 0x7F && read_ok) {
        switch((field_id >> 5) & 7) {
        case 0:
            net_read_byte();
            break;
        case 1:
            net_read_short();
            break;
        case 2:
            net_read_int();
            break;
        case 3:
            net_read_float();
            break;
        case 4:
            net_free_string16(net_read_string16());
            break;
        case 5:
            net_read_short();
            net_read_byte();
            net_read_short();
            break;
        case 6:
            net_read_int();
            net_read_int();
            net_read_int();
            break;
        }
    }
}

static struct ni_wi_payload read_window_items_payload(void)
{
    struct ni_wi_payload item;
    item.item_id = net_read_short();
    if(item.item_id != -1) {
        item.count = net_read_byte();
        item.metadata = net_read_short();
    }
    return item;
}

static bool net_read_packet(void)
{
    ubyte packet_id;

    // consume the old bytes
    if(read_ok)
        recv(sockfd, read_buffer, total_read, 0);
    total_read = 0;

    // try to read a packet
    packet_id = net_read_byte();
    if(!read_ok) {
        return false;
    }

#define UBYTE(name)                this.name = (ubyte) net_read_byte();
#define BYTE(name)                 this.name = net_read_byte();
#define SHORT(name)                this.name = net_read_short();
#define INT(name)                  this.name = net_read_int();
#define LONG(name)                 this.name = net_read_long();
#define FLOAT(name)                this.name = net_read_float();
#define DOUBLE(name)               this.name = net_read_double();
#define STRING8(name)              this.name = net_read_string8();
#define STRING16(name)             this.name = net_read_string16();
#define BOOL(name)                 this.name = net_read_byte();
#define METADATA(name)             /*this.name = */read_entity_metadata();
#define WINDOW_ITEMS_PAYLOAD(name) this.name = read_window_items_payload();

#define OPT(cond, stuff) if(cond) { stuff }
// todo: wrapper for alloca in case of bad size
#define BUF(type, name, size) this.name = alloca((size) * sizeof(*this.name)); for(size_t i = 0; i < (size_t) (size); i++) type(name[i])

#define PACKET(id, name, stuff) case id: { pkt_ ## name this = {0}; stuff; net_handle_pkt_ ## name(this); return true; }

    switch(packet_id) {
#include "packets_def.h"
    case 0x00: { // keep alive packet
        net_write_byte(0x00);
        return true;
    }
    default: {
        con_printf("unknown packet_id %hhd (0x%hhx)! disconnecting\n", packet_id, packet_id);
        net_shutdown();
        return false;
    }
    }

    return true;
}

void net_shutdown(void)
{
    init_ok = false;
    con_printf("net shutting down...\n");

    if(sockfd != INVALID_SOCKET) {
        closesocket(sockfd);
        sockfd = INVALID_SOCKET;
    }

#ifdef _WIN32
    WSACleanup();
#endif

    cl.state = cl_disconnected;
    vid_lock_fps();
}

void net_read_buf(void *dest, size_t n)
{
    ssize_t n_read;

    if(!init_ok || !dest || n == 0)
        return;

    // peek does not consume the bytes
    // we save them in a buffer in case we are missing some for an entire packet
    // if we are missing some then we can try to read them during the next frame
    n_read = recv(sockfd, read_buffer, total_read + n, MSG_PEEK);

    if(n_read == 0) {
        // EOF - todo: disonnect here? read recv man page and see return value 0
        read_ok = false;
        longjmp(read_abort, 1);
    } else if(n_read == -1) {
        read_ok = false;
        // ERR_TRYAGAIN is a common when using non-blocking sockets
        // it just means no data is currently available
        if(net_errno != ERR_TRYAGAIN && net_errno != ERR_WOULDBLOCK)
            perror("net_read_buf");
        longjmp(read_abort, 1);
    } else if((size_t) n_read - total_read != n) {
        // read some, but not everything
        // delay until next frame
        read_ok = false;
        total_read = n_read;
        longjmp(read_abort, 1);
    } else {
        read_ok = true;
        memcpy(dest, read_buffer + total_read, n);
        total_read = n_read;
    }
}

byte net_read_byte(void)
{
    byte b;
    net_read_buf(&b, 1);
    return b;
}

short net_read_short(void)
{
    short s;
    net_read_buf(&s, 2);
    return SDL_SwapBE16(s);
}

int net_read_int(void)
{
    int i;
    net_read_buf(&i, 4);
    return SDL_SwapBE32(i);
}

int64_t net_read_long(void)
{
    int64_t l;
    net_read_buf(&l, 8);
    return SDL_SwapBE64(l);
}

float net_read_float(void)
{
    union {
        float f;
        int i;
    } u;
    u.i = net_read_int();
    return u.f;
}

double net_read_double(void)
{
    union {
        double d;
        int64_t l;
    } u;
    u.l = net_read_long();
    return u.d;
}

bool net_read_bool(void)
{
    return net_read_byte() != 0;
}

string8 net_read_string8(void)
{
    string8 s;

    s.length = net_read_short();
    s.data = mem_alloc(s.length + 1);
    net_read_buf(s.data, s.length);
    s.data[s.length] = 0;

    return s;
}

string16 net_read_string16(void)
{
    string16 s;
    short size;
    int i;

    s.length = net_read_short();
    size = (s.length + 1) * sizeof(*s.data);
    s.data = mem_alloc(size);
    for(i = 0; i < s.length; i++)
        s.data[i] = net_read_short();
    s.data[s.length] = 0;

    return s;
}

void net_free_string8(string8 v)
{
    mem_free(v.data);
}

void net_free_string16(string16 v)
{
    mem_free(v.data);
}

string8 net_make_string8(const char *text)
{
    string8 str;
    str.length = strlen(text);
    str.data = mem_alloc(str.length + 1);
    strlcpy(str.data, text, str.length);
    return str;
}

string16 net_make_string16(const char *text)
{
    string16 str;
    int i;
    str.length = strlen(text);
    str.data = mem_alloc((str.length + 1) * sizeof(*str.data));
    for(i = 0; i < str.length; i++) {
        str.data[i] = (char16_t) text[i];
    }
    str.data[i] = u'\0';
    return str;
}

void net_write_buf(const void *buf, size_t n)
{
    ssize_t n_written;

    if(!init_ok || !buf || n == 0 || (cl.state == cl_disconnected && !should_disconnect))
        return;

    n_written = send(sockfd, buf, n, MSG_NOSIGNAL);
    if(n_written == -1) {
        if(net_errno == ERR_TRYAGAIN || net_errno == ERR_WOULDBLOCK) {
            // is this right?
            con_printf("net_write_buf: operation WILL block\n");
            setblocking(sockfd, true);
            net_write_buf(buf, n);
            setblocking(sockfd, false);
        } else {
            perror("net_write_buf");
            net_shutdown();
            return;
        }
    } else if((size_t) n_written != n) {
        // is this right?
        con_printf("net_write_buf: wrote only %ld/%lu bytes, gonna block\n", n_written, n);
        setblocking(sockfd, true);
        send(sockfd, (byte *) buf + n_written, n - n_written, MSG_NOSIGNAL);
        setblocking(sockfd, false);
    }
}

void net_write_byte(ubyte v)
{
    net_write_buf(&v, 1);
}

void net_write_short(short v)
{
    v = SDL_SwapBE16(v);
    net_write_buf(&v, 2);
}

void net_write_int(int v)
{
    v = SDL_SwapBE32(v);
    net_write_buf(&v, 4);
}

void net_write_long(int64_t v)
{
    v = SDL_SwapBE64(v);
    net_write_buf(&v, 8);
}

void net_write_float(float v)
{
    union {
        float f;
        int i;
    } u;
    u.f = v;
    net_write_int(u.i);
}

void net_write_double(double v)
{
    union {
        double d;
        int64_t l;
    } u;
    u.d = v;
    net_write_long(u.l);
}

void net_write_bool(bool v)
{
    net_write_byte(v);
}

void net_write_string8(string8 v)
{
    net_write_short(v.length);
    net_write_buf(v.data, v.length);
}

void net_write_string16(string16 v)
{
    short len = v.length;
    net_write_short(len);
    for(int i = 0; i < len; i++) {
        net_write_short((short) v.data[i]);
    }
}

void connect_f(void)
{
    struct addrinfo hints = {0}, *info;
    char *addrstr, *p;
    int port, err;

    if(cmd_argc() != 2) {
        con_printf("usage: %s <ip>[:<port>]\n", cmd_argv(0));
    }

    addrstr = cmd_argv(1);

    if(cl.state != cl_disconnected) {
        con_printf("disconnect first\n");
        return;
    }

    p = addrstr;
    while(*p != '\0' && *p != ':')
        p++;

    if(*p == '\0') {
        // no port specified
        port = 25565;
    } else {
        port = strtol(p, NULL, 10);
        if(net_errno == EINVAL) {
            con_printf("invalid ip\n");
            return;
        }
        *p = 0; // set to 0 for ip address parsing (idk if needed)
    }

    // todo: ipv6 support if you can even use that with a beta server
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    if((err = getaddrinfo(addrstr, "http", &hints, &info))) {
        con_printf("error: %s\n", gai_strerror(err));
        return;
    }

    if(info != NULL) {
        net_init();
        net_connect((struct sockaddr_in *) info->ai_addr, port);
    }

    freeaddrinfo(info);
}

void disconnect_f(void)
{
    if(cl.state != cl_disconnected) {
        vid_lock_fps();
        cl_end_game();

        // net_update will actually disconnect next update
        should_disconnect = true;
        cl.state = cl_disconnected;

        con_show();
    }
}

void say_f(void)
{
    char *msg;
    string16 message;

    if(cmd_argc() == 1) {
        con_printf("usage: %s <message>\n", cmd_argv(0));
    }

    if(cl.state != cl_connected) {
        con_printf("can't \"%s\", not connected\n", cmd_argv(0));
        return;
    }

    msg = cmd_args(1, cmd_argc());
    message = net_make_string16(msg);

    net_write_pkt_chat_message((pkt_chat_message) {
        .message = message
    });

    net_free_string16(message);
}

void respawn_f(void)
{
    if(cl.state != cl_connected) {
        con_printf("can't \"%s\", not connected\n", cmd_argv(0));
        return;
    }
    net_write_pkt_respawn((pkt_respawn) {
        .dimension = 0
    });
}
