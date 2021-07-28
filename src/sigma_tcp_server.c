#include "sigma_tcp/sigma_tcp_server.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <stdbool.h>

#include "logging.h"

#include <netinet/in.h>
#include <net/if.h>

static void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET)
		return &(((struct sockaddr_in*)sa)->sin_addr);

	return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

#define COMMAND_READ_REQUEST 0x0A
#define COMMAND_READ_RESPONSE 0x0B
#define COMMAND_WRITE_REQUEST 0x09
#define COMMAND_WRITE_RESPONSE 0x0B
#define COMMAND_REG_ASAP_CONN 0x1B

#define READ_REQUEST_HEADER_LEN	8
#define WRITE_REQUEST_HEADER_LEN 10
#define REG_ASAP_CONN_LEN 11

static void handle_connection(int fd, struct backend_ops const *backend_ops)
{
	uint8_t *buf;
	size_t buf_size;
	uint8_t *p;
	uint16_t packet_len;
	unsigned int len;
	unsigned int addr;
	uint8_t ic_num;
	int count, ret;
	char command;
	
	(void)ic_num;

	count = 0;

	buf_size = 256;
	buf = malloc(buf_size);
	if (!buf)
		goto exit;

	p = buf;

	while (1) {
		memmove(buf, p, count);  // discard processed packet data
		p = buf + count;

		ret = read(fd, p, buf_size - count);

		if (ret <= 0)
			break;

		p = buf;

		count += ret;

		while (count >= READ_REQUEST_HEADER_LEN) {
			command = p[0];
			if (command == COMMAND_READ_REQUEST) {
				packet_len = (p[1] << 8) | p[2];
				ic_num = p[3];
				len = (p[4] << 8) | p[5];
				addr = (p[6] << 8) | p[7];

				p += READ_REQUEST_HEADER_LEN;
				count -= READ_REQUEST_HEADER_LEN;

			    /* LOG_INFO("received read command (0x%02X) packet_len: %i, IC: %X len: %i addr: 0x%04X", command, packet_len, ic_num, len, addr); */

				if ((ret = adau_read(backend_ops, addr, len, buf + 4)) < 0) {
					/* LOG_ERROR("read returned %i (%s)", ret, strerror(errno)); */
				}

				buf[0] = COMMAND_READ_RESPONSE;
				buf[1] = (0x4 + len) >> 8;
				buf[2] = (0x4 + len) & 0xff;
				buf[3] = ret;
				write(fd, buf, 4 + len);
			} else if (command == COMMAND_WRITE_REQUEST) {
				packet_len = (p[3] << 8) | p[4];
				ic_num = p[5];
				len = (p[6] << 8) | p[7];
				addr = (p[8] << 8) | p[9];

				/* LOG_INFO("processing write command (0x%02X) IC: %X packet_len: %i received %i", command, ic_num, packet_len, count); */

				/* not enough data, fetch next bytes */
				if (count < packet_len) {
					if (buf_size < packet_len) {
						/* LOG_INFO("reallocating packet buffer with size: %i", packet_len); */
						uint8_t *new_buf = malloc(packet_len);
						if (!new_buf)
							goto exit;
						memcpy(new_buf, p, count);
						free(buf);
						buf_size = packet_len;
						buf = new_buf;
						p = buf;
					}
					break;
				}
				else {
					/* LOG_INFO("received write command packet len: %i addr: 0x%04X", len, addr); */

					p += WRITE_REQUEST_HEADER_LEN;
					count -= WRITE_REQUEST_HEADER_LEN;

					if ((ret = adau_write(backend_ops, addr, len, p)) < 0) {
						/* LOG_ERROR("backend write returned %i (%s)", ret, strerror(errno)); */
					}
					
					p += len;
					count -= len;
				}
			}
#ifdef SIGMA_TCP_EEPROM_PROGRAM
			else if (command == COMMAND_REG_ASAP_CONN)
			{
				if (count >= REG_ASAP_CONN_LEN)
				{
					p += REG_ASAP_CONN_LEN;
					count -= REG_ASAP_CONN_LEN;

					if ((ret = adau_write_to_eeprom()) < 0)
					{
						LOG_ERROR("EEPROM write failed");
					}
				}
			}
#endif /* SIGMA_TCP_EEPROM_PROGRAM */
			else {
				LOG_ERROR("unrecognized command: 0x%02X", command);
                goto exit;
			}
		}
	}

exit:
	LOG_INFO("exiting");
	free(buf);
}

#define PORT 8086

int sigma_tcp_server_listen(struct backend_ops const *backend)
{
    int sockfd, new_fd;
	struct sockaddr_storage their_addr;
    socklen_t sin_size;
    char s[INET6_ADDRSTRLEN];
    int ip_protocol = 0;
    int ret = 0;

    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&their_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(PORT);
    ip_protocol = IPPROTO_IP;

    sockfd = socket(AF_INET, SOCK_STREAM, ip_protocol);
    if (sockfd < 0) {
        perror("server: socket");
        ret = -1;
        goto CLEAN_UP;
    }

    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    int err = bind(sockfd, (struct sockaddr *)&their_addr, sizeof(their_addr));
    if (err != 0) {
        LOG_ERROR("Socket unable to bind: errno %d", errno);
        ret = -1;
        goto CLEAN_UP;
    }
    LOG_INFO("Socket bound, port %d", PORT);

    err = listen(sockfd, 1);
    if (err != 0) {
        LOG_ERROR("Error occurred during listen: errno %d", errno);
        ret = -1;
        goto CLEAN_UP;
    }

    LOG_INFO("Waiting for connections...");
	
    while (true) {
        sin_size = sizeof their_addr;
        new_fd = accept(sockfd, (struct sockaddr *)&their_addr, &sin_size);
        if (new_fd == -1) {
            perror("accept");
            continue;
        }

        inet_ntop(their_addr.ss_family,
            get_in_addr((struct sockaddr *)&their_addr),
            s, sizeof s);

        LOG_INFO("New connection from %s", s);
		handle_connection(new_fd, backend);
        close(new_fd);
        LOG_INFO("Connection closed");
    }

CLEAN_UP:
    close(sockfd);
    return ret;
}