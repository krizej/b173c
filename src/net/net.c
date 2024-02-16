#include <SDL2/SDL_endian.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include "common.h"
#include "net.h"
#include "net_internal.h"
#include "client/client.h"
#include "client/console.h"
#include "vid/vid.h"
#include <setjmp.h>
#include <uchar.h>
#include "packets.h"
#include <string.h>

#define PACKET(id, name, stuff) [id] = #name,
const char *packet_names[256] = {
#include "packets_def.h"
};
#undef PACKET

static bool init_ok = false;
static bool cmds_registered = false;
static bool should_disconnect = false;

static bool read_ok = false;
static size_t total_read = 0;
static byte read_buffer[32*1024] = {0};
jmp_buf read_abort;

static int sockfd = -1;

#define perror(desc) con_printf("%s: %s\n", desc, strerror(errno))

static void setblocking(int fd, bool blocking)
{
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
}

void disconnect_f(void);
void connect_f(void);
void say_f(void);
void respawn_f(void);

void net_init(void)
{
	/* do not reinitialize */
	if(init_ok)
		return;

	/* open ipv4 tcp stream socket */
	if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		perror("socket");
		net_shutdown(); // deinit
		return;
	}

	/* set socket to nonblocking */
	setblocking(sockfd, false);

	/* register net-related commands */
	if(!cmds_registered) {
		cmd_register("connect", connect_f);
		cmd_register("disconnect", disconnect_f);
		cmd_register("say", say_f);
		cmd_register("respawn", respawn_f);
        cmds_registered = true;
	}

	init_ok = true;
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

	// connect to server
	// set socket to blocking for this because its easier this way
	setblocking(sockfd, true);
	if(connect(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("connect");
		net_shutdown(); // deinit
		return;
	}
	setblocking(sockfd, false);

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

		while(net_read_packet())
			;

		net_write_packets();
	}

	// disconnect if requested
	if(should_disconnect) {
		net_connect(NULL, 0);
		cl.state = cl_disconnected;
		vid_lock_fps(); // no need to exhaust the cpu by drawing the menu in 1 billion fps
		net_write_packets(); // reset handshake flag
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
		read(sockfd, read_buffer, total_read);
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

	// close socket
	setblocking(sockfd, true);
	if(sockfd != -1) {
		if(close(sockfd) < 0) {
			perror("close");
			// bruh :|
		}
		sockfd = -1;
	}

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
	n_read = recv(sockfd, read_buffer, total_read+n, MSG_PEEK);

	if(n_read == 0) {
		// EOF - todo: disonnect here? read recv man page and see return value 0
		read_ok = false;
		longjmp(read_abort, 1);
	} else if(n_read == -1) {
		read_ok = false;
		// EAGAIN is a common when using non-blocking sockets
		// it just means no data is currently available
		if(errno != EAGAIN)
			con_printf("net_read_buf: %s (error %d)\n", strerror(errno), errno);
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

long net_read_long(void)
{
	long l;
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
		long l;
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

	if(!init_ok || !buf || n == 0 || cl.state == cl_disconnected)
		return;

	n_written = send(sockfd, buf, n, MSG_NOSIGNAL);
	if(n_written == -1) {
		if(errno == EAGAIN || errno == EWOULDBLOCK) {
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
		write(sockfd, (byte *)buf + n_written, n - n_written);
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

void net_write_long(long v)
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
		long l;
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
		net_write_short(v.data[i]);
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
		if(errno == EINVAL) {
			con_printf("invalid ip\n");
			return;
		}
		*p = 0; // set to 0 for ip address parsing (probably needed but idk)
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
	return;
}

void disconnect_f(void)
{
	if(cl.state != cl_disconnected) {
		if(cl.state == cl_connected) {
            string16 reason;
			// set to blocking so the disconnect packet gets sent
			// before the socket is closed
			setblocking(sockfd, true);
            reason = net_make_string16("Quitting");
            net_write_pkt_disconnect((pkt_disconnect) {
                .reason = reason
            });
            net_free_string16(reason);
			setblocking(sockfd, false);
		}

		// todo: world_cleanup() or something
		// or just move world vars to cl.game
		memset(&cl.game, 0, sizeof(cl.game));

		// net_update will disconnect next update
		cl.state = cl_disconnected;
		vid_lock_fps();
		should_disconnect = true;

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

// fixme: this is temporary (or maybe leave it for dat sweet customizability and just make the respawn button execute this command)
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
