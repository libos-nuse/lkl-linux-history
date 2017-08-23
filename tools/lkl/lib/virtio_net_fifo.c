/*
 * POSIX file descriptor(in and out) based 
 * virtual network interface feature for LKL
 */
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <limits.h>
#include <fcntl.h>
#include <sys/poll.h>
#include <sys/uio.h>

#include "virtio.h"

struct lkl_netdev_fifo {
	struct lkl_netdev dev;
	/* file-descriptor based device */
	int fd_rx;
	int fd_tx;
	/*
	 * Controlls the poll mask for fd. Can be acccessed concurrently from
	 * poll, tx, or rx routines but there is no need for syncronization
	 * because:
	 *
	 * (a) TX and RX routines set different variables so even if they update
	 * at the same time there is no race condition
	 *
	 * (b) Even if poll and TX / RX update at the same time poll cannot
	 * stall: when poll resets the poll variable we know that TX / RX will
	 * run which means that eventually the poll variable will be set.
	 */
	int poll_tx, poll_rx;
	/* controle pipe */
	int pipe[2];
	/* mode switch */
	int ascii :1;
};

/* Code from https://github.com/sora/ethpipe/blob/master/software/driver/ethpipe.c */
#define _SKP    0x20

static const unsigned char _atob[] = {
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 0-7 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 8-15 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 16-23 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 24-31 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 32-39 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 40-47 */
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,     /* 48-55 */
	0x08, 0x09, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 56-63 */
	_SKP, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, _SKP,     /* 64-71 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 72-79 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 80-87 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 88-95 */
	_SKP, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, _SKP,     /* 96-103 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 104-111 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 112-119 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 120-127 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 128-135 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 136-143 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 144-151 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 152-159 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 160-167 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 168-175 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 176-183 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 184-191 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 192-199 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 200-207 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 208-215 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 216-223 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 224-231 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 232-239 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP,     /* 240-247 */
	_SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP, _SKP };   /* 248-255 */

static const short _btoa[257] = {
	0x3030, 0x3130, 0x3230, 0x3330, 0x3430, 0x3530, 0x3630, 0x3730,     /* 0-7 */
	0x3830, 0x3930, 0x4130, 0x4230, 0x4330, 0x4430, 0x4530, 0x4630,     /* 8-15 */
	0x3031, 0x3131, 0x3231, 0x3331, 0x3431, 0x3531, 0x3631, 0x3731,     /* 16-23 */
	0x3831, 0x3931, 0x4131, 0x4231, 0x4331, 0x4431, 0x4531, 0x4631,     /* 24-31 */
	0x3032, 0x3132, 0x3232, 0x3332, 0x3432, 0x3532, 0x3632, 0x3732,     /* 32-39 */
	0x3832, 0x3932, 0x4132, 0x4232, 0x4332, 0x4432, 0x4532, 0x4632,     /* 40-47 */
	0x3033, 0x3133, 0x3233, 0x3333, 0x3433, 0x3533, 0x3633, 0x3733,     /* 48-55 */
	0x3833, 0x3933, 0x4133, 0x4233, 0x4333, 0x4433, 0x4533, 0x4633,     /* 56-63 */
	0x3034, 0x3134, 0x3234, 0x3334, 0x3434, 0x3534, 0x3634, 0x3734,     /* 64-71 */
	0x3834, 0x3934, 0x4134, 0x4234, 0x4334, 0x4434, 0x4534, 0x4634,     /* 72-79 */
	0x3035, 0x3135, 0x3235, 0x3335, 0x3435, 0x3535, 0x3635, 0x3735,     /* 80-87 */
	0x3835, 0x3935, 0x4135, 0x4235, 0x4335, 0x4435, 0x4535, 0x4635,     /* 88-95 */
	0x3036, 0x3136, 0x3236, 0x3336, 0x3436, 0x3536, 0x3636, 0x3736,     /* 96-103 */
	0x3836, 0x3936, 0x4136, 0x4236, 0x4336, 0x4436, 0x4536, 0x4636,     /* 104-111 */
	0x3037, 0x3137, 0x3237, 0x3337, 0x3437, 0x3537, 0x3637, 0x3737,     /* 112-119 */
	0x3837, 0x3937, 0x4137, 0x4237, 0x4337, 0x4437, 0x4537, 0x4637,     /* 120-127 */
	0x3038, 0x3138, 0x3238, 0x3338, 0x3438, 0x3538, 0x3638, 0x3738,     /* 128-135 */
	0x3838, 0x3938, 0x4138, 0x4238, 0x4338, 0x4438, 0x4538, 0x4638,     /* 136-143 */
	0x3039, 0x3139, 0x3239, 0x3339, 0x3439, 0x3539, 0x3639, 0x3739,     /* 144-151 */
	0x3839, 0x3939, 0x4139, 0x4239, 0x4339, 0x4439, 0x4539, 0x4639,     /* 152-159 */
	0x3041, 0x3141, 0x3241, 0x3341, 0x3441, 0x3541, 0x3641, 0x3741,     /* 160-167 */
	0x3841, 0x3941, 0x4141, 0x4241, 0x4341, 0x4441, 0x4541, 0x4641,     /* 168-175 */
	0x3042, 0x3142, 0x3242, 0x3342, 0x3442, 0x3542, 0x3642, 0x3742,     /* 176-183 */
	0x3842, 0x3942, 0x4142, 0x4242, 0x4342, 0x4442, 0x4542, 0x4642,     /* 184-191 */
	0x3043, 0x3143, 0x3243, 0x3343, 0x3443, 0x3543, 0x3643, 0x3743,     /* 192-199 */
	0x3843, 0x3943, 0x4143, 0x4243, 0x4343, 0x4443, 0x4543, 0x4643,     /* 200-207 */
	0x3044, 0x3144, 0x3244, 0x3344, 0x3444, 0x3544, 0x3644, 0x3744,     /* 208-215 */
	0x3844, 0x3944, 0x4144, 0x4244, 0x4344, 0x4444, 0x4544, 0x4644,     /* 216-223 */
	0x3045, 0x3145, 0x3245, 0x3345, 0x3445, 0x3545, 0x3645, 0x3745,     /* 224-231 */
	0x3845, 0x3945, 0x4145, 0x4245, 0x4345, 0x4445, 0x4545, 0x4645,     /* 232-239 */
	0x3046, 0x3146, 0x3246, 0x3346, 0x3446, 0x3546, 0x3646, 0x3746,     /* 240-247 */
	0x3846, 0x3946, 0x4146, 0x4246, 0x4346, 0x4446, 0x4546, 0x4646 };   /* 248-255 */


static int btoa(int fd, void *data, size_t len)
{
	unsigned char *ptr = data;
	int ret, written = 0;
	char *buf;

	buf = malloc(14*2 + 3 + (len - 14)*3 + 2);
	if (!buf) {
		fprintf(stderr, "fifo(btoa): failed to allocate memory (len=%ld)\n", len);
		return -1;
	}

	/* 1: L2 header */
	written += snprintf(buf, LKL_ETH_ALEN * 2 + 2,
			    "%02X%02X%02X%02X%02X%02X ",
			    ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5]);
	ptr += LKL_ETH_ALEN;
	written += snprintf(buf + written, LKL_ETH_ALEN * 2 + 2,
			    "%02X%02X%02X%02X%02X%02X ",
			    ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5]);
	ptr += LKL_ETH_ALEN;
	written += snprintf(buf + written, 2*2 + 2, "%02X%02X ",
			    ptr[0], ptr[1]);
	ptr += 2;

	/* 2: rest of packets */
	while (ptr < (unsigned char *)data + len) {
		written += snprintf(buf + written, 2*1 + 1 + 2, "%02X ",
			    ptr[0]);
		ptr++;
	}

	written += snprintf(buf + written, 2, "\n");
	ret = write(fd, buf, written);
	free(buf);

	return ret;
}

static int fifo_net_tx(struct lkl_netdev *nd, struct iovec *iov, int cnt)
{
	int ret = 0, i;
	struct lkl_netdev_fifo *nd_fifo =
		container_of(nd, struct lkl_netdev_fifo, dev);

	if (!nd_fifo->ascii)
		do {
			ret = writev(nd_fifo->fd_tx, iov, cnt);
		} while (ret == -1 && errno == EINTR);
	else
		for (i = 0; i < cnt; i++) {
			if (iov[i].iov_len == 0)
				continue;
			ret += btoa(nd_fifo->fd_tx, iov[i].iov_base,
				   iov[i].iov_len);
		}

	if (ret < 0) {
		if (errno != EAGAIN) {
			perror("write to fd netdev fails");
		} else {
			char tmp;

			nd_fifo->poll_tx = 1;
			if (write(nd_fifo->pipe[1], &tmp, 1) <= 0)
				perror("virtio net fd pipe write");
		}
	}
	return ret;
}

static int atob(void *data, size_t len, struct iovec *iov)
{
	unsigned char *ptr = data;
	int rlen = 0;
	unsigned char byte;
	char buf[3] = {0, 0, 0};

	while (len > 0) {
		if (*ptr == ' ') {
			ptr++;
			continue;
		}

		if (*ptr == '\n')
			break;

		buf[0] = *ptr;
		buf[1] = *(ptr + 1);

		byte = strtoul(buf, NULL, 16);
		memcpy(iov->iov_base + rlen, &byte, 1);

		ptr += 2;
		len -= 2; rlen++;
	}

#ifdef DEBUG
	int i =0;
	*ptr = '\0';
	fprintf(stderr, "==== origin=%s===\n", (char *)data);
	fprintf(stderr, "==== rlen=%d len=%ld ", rlen, len);
	for (i = 0; i < rlen; i++) {
		fprintf(stderr, "%02X ", *((unsigned char *)iov->iov_base + i));
	}
	fprintf(stderr, "=====\n");
#endif

	iov->iov_len = rlen;
	return rlen;
}

static int fifo_net_rx(struct lkl_netdev *nd, struct iovec *iov, int cnt)
{
	int ret = 0;
	struct lkl_netdev_fifo *nd_fifo =
		container_of(nd, struct lkl_netdev_fifo, dev);

	if (!nd_fifo->ascii)
		do {
			ret = readv(nd_fifo->fd_rx, (struct iovec *)iov, cnt);
		} while (ret == -1 && errno == EINTR);
	else {
		char rbuf[3000]; /* XXX */

		ret = read(nd_fifo->fd_rx, rbuf, sizeof(rbuf));
		if (ret > 0)
			ret = atob(rbuf, ret, &iov[1]);
	}

	if (ret < 0) {
		if (errno != EAGAIN) {
			perror("virtio net fd read");
		} else {
			char tmp;

			nd_fifo->poll_rx = 1;
			if (write(nd_fifo->pipe[1], &tmp, 1) < 0)
				perror("virtio net fd pipe write");
		}
	}
	return ret;
}

static int fifo_net_poll(struct lkl_netdev *nd)
{
	struct lkl_netdev_fifo *nd_fifo =
		container_of(nd, struct lkl_netdev_fifo, dev);
	struct pollfd pfds[3] = {
		{
			.fd = nd_fifo->fd_rx,
		},
		{	.fd = nd_fifo->fd_tx,
		},
		{
			.fd = nd_fifo->pipe[0],
			.events = POLLIN,
		},
	};
	int ret;

	if (nd_fifo->poll_rx)
		pfds[0].events |= POLLIN|POLLPRI;
	if (nd_fifo->poll_tx)
		pfds[1].events |= POLLOUT;

	do {
		ret = poll(pfds, 3, -1);
	} while (ret == -1 && errno == EINTR);

	if (ret < 0) {
		perror("virtio net fd poll");
		return 0;
	}

	if (pfds[2].revents & (POLLHUP|POLLNVAL))
		return LKL_DEV_NET_POLL_HUP;

	if (pfds[2].revents & POLLIN) {
		char tmp[PIPE_BUF];

		ret = read(nd_fifo->pipe[0], tmp, PIPE_BUF);
		if (ret == 0)
			return LKL_DEV_NET_POLL_HUP;
		if (ret < 0)
			perror("virtio net fd pipe read");
	}

	ret = 0;

	if (pfds[0].revents & (POLLIN|POLLPRI)) {
		nd_fifo->poll_rx = 0;
		ret |= LKL_DEV_NET_POLL_RX;
	}

	if (pfds[1].revents & POLLOUT) {
		nd_fifo->poll_tx = 0;
		ret |= LKL_DEV_NET_POLL_TX;
	}

	return ret;
}

static void fifo_net_poll_hup(struct lkl_netdev *nd)
{
	struct lkl_netdev_fifo *nd_fifo =
		container_of(nd, struct lkl_netdev_fifo, dev);

	/* this will cause a POLLHUP / POLLNVAL in the poll function */
	close(nd_fifo->pipe[0]);
	close(nd_fifo->pipe[1]);
}

static void fifo_net_free(struct lkl_netdev *nd)
{
	struct lkl_netdev_fifo *nd_fifo =
		container_of(nd, struct lkl_netdev_fifo, dev);

	close(nd_fifo->fd_rx);
	close(nd_fifo->fd_tx);
	free(nd_fifo);
}

struct lkl_dev_net_ops fifo_net_ops =  {
	.tx = fifo_net_tx,
	.rx = fifo_net_rx,
	.poll = fifo_net_poll,
	.poll_hup = fifo_net_poll_hup,
	.free = fifo_net_free,
};

struct lkl_netdev *lkl_register_netdev_fifo(int fd_rx, int fd_tx, int ascii)
{
	struct lkl_netdev_fifo *nd;

	nd = malloc(sizeof(*nd));
	if (!nd) {
		fprintf(stderr, "fdnet: failed to allocate memory\n");
		/* TODO: propagate the error state, maybe use errno for that? */
		return NULL;
	}

	memset(nd, 0, sizeof(*nd));

	nd->fd_rx = fd_rx;
	nd->fd_tx = fd_tx;
	if (pipe(nd->pipe) < 0) {
		perror("pipe");
		free(nd);
		return NULL;
	}

	if (fcntl(nd->pipe[0], F_SETFL, O_NONBLOCK) < 0) {
		perror("fnctl");
		close(nd->pipe[0]);
		close(nd->pipe[1]);
		free(nd);
		return NULL;
	}

	nd->ascii = ascii;
	nd->dev.ops = &fifo_net_ops;
	return &nd->dev;
}

/* TODO support offload, if possible */
struct lkl_netdev *lkl_netdev_fifo_create(char *ifname, int ascii)
{
	struct lkl_netdev *nd;
	int fd_rx, fd_tx;

	char *ifname_rx = NULL, *ifname_tx = NULL;

	ifname_rx = strtok(ifname, "|");
	if(ifname_rx == NULL){
		fprintf(stderr, "parse ifname of fifo\n");
		return NULL;
	}
	ifname_tx = strtok(NULL, "|");
	if(ifname_tx == NULL){
		fprintf(stderr, "parse ifname of fifo\n");
		return NULL;
	}
	if(strtok(NULL, "|") != NULL){
		fprintf(stderr, "parse ifname of fifo\n");
		return NULL;
	}

	fd_rx = open(ifname_rx, O_RDWR|O_NONBLOCK);
	if (fd_rx < 0) {
		perror("open rx fifo");
		fprintf(stderr, "ifname_rx = %s\n", ifname_rx);
		return NULL;
	}

	fd_tx = open(ifname_tx, O_RDWR|O_NONBLOCK);
	if (fd_tx < 0) {
		perror("open tx fifo");
		fprintf(stderr, "ifname_tx = %s\n", ifname_tx);
		return NULL;
	}

	nd = lkl_register_netdev_fifo(fd_rx, fd_tx, ascii);
	if (!nd) {
		perror("failed to register to.");
		close(fd_rx);
		close(fd_tx);
		return NULL;
	}

	return nd;
}
