/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <init.h>

#if defined(CONFIG_BSD_LIBRARY) && defined(CONFIG_NET_SOCKETS_OFFLOAD)
#include <net/socket.h>
#endif

#include <logging/log.h>
LOG_MODULE_REGISTER(board_nonsecure, CONFIG_BOARD_LOG_LEVEL);

#define AT_CMD_MAX_READ_LENGTH	128
#define AT_CMD_LEN(cmd)		(sizeof (cmd) - 1)
#define AT_CMD_MAGPIO		"AT%XMAGPIO=1,1,1,6,0,746,803,1,698,748," \
				"1,1710,2200,2,824,894,3,880,960,4,791,849"
#define AT_CMD_COEX0		"AT%XCOEX0=1,1,1565,1586"
#define AT_CMD_TRACE		"AT%XMODEMTRACE=0"

static int orion_magpio_configure(void)
{
#if defined(CONFIG_BSD_LIBRARY) && defined(CONFIG_NET_SOCKETS_OFFLOAD)
	int at_socket_fd;
	int buffer;
	uint8_t read_buffer[AT_CMD_MAX_READ_LENGTH];

	if (!IS_ENABLED(CONFIG_BSD_LIBRARY_SYS_INIT)) {
		LOG_INF("BSDlib is not yet initialized, AT commands not sent");
		LOG_INF("Configuration of MAGPIO and COEX0 is left to drivers");
		return 0;
	}

	at_socket_fd = socket(AF_LTE, SOCK_DGRAM, NPROTO_AT);
	if (at_socket_fd == -1) {
		LOG_ERR("AT socket could not be opened");
		return -EFAULT;
	}

	LOG_DBG("AT CMD: %s", log_strdup(AT_CMD_TRACE));
	buffer = send(at_socket_fd, AT_CMD_TRACE, AT_CMD_LEN(AT_CMD_TRACE), 0);
	if (buffer != AT_CMD_LEN(AT_CMD_TRACE)) {
		LOG_ERR("XMODEMTRACE command failed");
		close(at_socket_fd);
		__ASSERT_NO_MSG(false);
		return -EIO;
	}

	buffer = recv(at_socket_fd, read_buffer, AT_CMD_MAX_READ_LENGTH, 0);
	LOG_DBG("AT RESP: %s", log_strdup(read_buffer));
	if ((buffer < 2) ||
	    (memcmp("OK", read_buffer, 2 != 0))) {
		LOG_ERR("XMODEMTRACE received unexpected response");
		close(at_socket_fd);
		__ASSERT_NO_MSG(false);
		return -EIO;
	}

	LOG_DBG("AT CMD: %s", log_strdup(AT_CMD_MAGPIO));
	buffer = send(at_socket_fd, AT_CMD_MAGPIO,
		      AT_CMD_LEN(AT_CMD_MAGPIO), 0);
	if (buffer != AT_CMD_LEN(AT_CMD_MAGPIO)) {
		LOG_ERR("MAGPIO command failed");
		close(at_socket_fd);
		return -EIO;
	}

	LOG_DBG("AT CMD: %s", log_strdup(AT_CMD_COEX0));
	buffer = send(at_socket_fd, AT_CMD_COEX0,
		      AT_CMD_LEN(AT_CMD_COEX0), 0);
	if (buffer != AT_CMD_LEN(AT_CMD_COEX0)) {
		LOG_ERR("COEX0 command failed");
		close(at_socket_fd);
		return -EIO;
	}

	buffer = recv(at_socket_fd, read_buffer, AT_CMD_MAX_READ_LENGTH, 0);
	LOG_DBG("AT RESP: %s", log_strdup(read_buffer));
	if ((buffer < 2) ||
	    (memcmp("OK", read_buffer, 2 != 0))) {
		LOG_ERR("MAGPIO command failed");
		close(at_socket_fd);
		return -EIO;
	}

	LOG_DBG("MAGPIO successfully configured");

	close(at_socket_fd);
#endif
	return 0;
}

static int orion_board_init(const struct device *dev)
{
	int err;

	err = orion_magpio_configure();
	if (err) {
		LOG_ERR("orion_magpio_configure failed with error: %d", err);
		return err;
	}

	return 0;
}

SYS_INIT(orion_board_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
