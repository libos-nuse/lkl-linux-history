/*
 * netmap(VALE) descriptor based virtual network interface feature for
 * LKL Copyright (c) 2015,2016 Ryo Nakamura, Hajime Tazaki
 *
 * Author: Ryo Nakamura <upa@wide.ad.jp>
 *         Hajime Tazaki <thehajime@gmail.com>
 *         Octavian Purdila <octavian.purdila@intel.com>
 *
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

#ifndef NETMAP_WITH_LIBS
#define NETMAP_WITH_LIBS
#endif
#include <net/netmap_user.h>

#include "virtio.h"
#include "virtio_net_fd.h"

struct lkl_netdev_vale {
	struct lkl_netdev dev;
	/* netmap-descriptor based device */
	struct nm_desc *nmd;
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
};

static __inline int
netmap_writev(struct nm_desc *nmd, struct iovec *iov, int iovcnt)
{
	int r, i;
	int len = 0;

	for (r = nmd->cur_tx_ring; ; ) {
		struct netmap_ring *ring = NETMAP_TXRING(nmd->nifp, r);
		uint32_t cur, idx;
		char *buf;

		if (nm_ring_empty(ring)) {
			r++;
			if (r > nmd->last_tx_ring)
				r = nmd->first_tx_ring;
			if (r == nmd->cur_tx_ring)
				break;
			continue;
		}
		cur = ring->cur;
		idx = ring->slot[cur].buf_idx;
		buf = NETMAP_BUF(ring, idx);

		for (i = 0; i < iovcnt; i++) {
			if (len + iov[i].iov_len > ring->nr_buf_size)
				break;
			nm_pkt_copy(iov[i].iov_base, &buf[len], iov[i].iov_len);
			len += iov[i].iov_len;
		}
		ring->slot[cur].len = len;
		ring->head = ring->cur = nm_ring_next(ring, cur);
		nmd->cur_tx_ring = r;
		ioctl(nmd->fd, NIOCTXSYNC, NULL);
		break;
	}
	if(len == 0){
		len = -1;
		errno = EAGAIN;
	}
	return (len);
}

static int vale_net_tx(struct lkl_netdev *nd, struct iovec *iov, int cnt)
{
	int ret;
	struct lkl_netdev_vale *nd_vale =
		container_of(nd, struct lkl_netdev_vale, dev);

	do {
		ret = netmap_writev(nd_vale->nmd, iov, cnt);
	} while (ret == -1 && errno == EINTR);

	if (ret < 0) {
		if (errno != EAGAIN) {
			perror("write to fd netdev fails");
		} else {
			char tmp;

			nd_vale->poll_tx = 1;
			if (write(nd_vale->pipe[1], &tmp, 1) <= 0)
				perror("virtio net vale pipe write");
		}
	}
	return ret;
}

static __inline int
netmap_readv(struct nm_desc *nmd, struct iovec *iov, int iovcnt)
{
	int len = 0;
	int i = 0;
	int r;

	for (r = nmd->cur_rx_ring; ; ) {
		struct netmap_ring *ring = NETMAP_RXRING(nmd->nifp, r);
		uint32_t cur, idx;
		char *buf;
		size_t left;

		if (nm_ring_empty(ring)) {
			r++;
			if (r > nmd->last_rx_ring)
				r = nmd->first_rx_ring;
			if (r == nmd->cur_rx_ring)
				break;
			continue;
		}
		cur = ring->cur;
		idx = ring->slot[cur].buf_idx;
		buf = NETMAP_BUF(ring, idx);
		left = ring->slot[cur].len;

		for (i = 0; i < iovcnt && left > 0; i++) {
			if (iov[i].iov_len > left)
				iov[i].iov_len = left;
			memcpy(iov[i].iov_base, &buf[len], iov[i].iov_len);
			len += iov[i].iov_len;
			left -= iov[i].iov_len;
		}
		ring->head = ring->cur = nm_ring_next(ring, cur);
		nmd->cur_rx_ring = r;
		ioctl(nmd->fd, NIOCRXSYNC, NULL);
		break;
	}
	for (; i < iovcnt; i++)
		iov[i].iov_len = 0;

	if(len == 0){
		len = -1;
		errno = EAGAIN;
	}
	return (len);
}

/*
 *Maybe need change
 */
static int vale_net_rx(struct lkl_netdev *nd, struct iovec *iov, int cnt)
{
	int ret;
	struct lkl_netdev_vale *nd_vale =
		container_of(nd, struct lkl_netdev_vale, dev);

	do {
		ret = netmap_readv(nd_vale->nmd, (struct iovec *)iov, cnt);
	} while (ret == -1 && errno == EINTR);

	if (ret < 0) {
		if (errno != EAGAIN) {
			perror("virtio net vale read");
		} else {
			char tmp;

			nd_vale->poll_rx = 1;
			if (write(nd_vale->pipe[1], &tmp, 1) < 0)
				perror("virtio net vale pipe write");
		}
	}
	return ret;
}

static int vale_net_poll(struct lkl_netdev *nd)
{
	struct lkl_netdev_vale *nd_vale =
		container_of(nd, struct lkl_netdev_vale, dev);
	struct pollfd pfds[2] = {
		{
			.fd = nd_vale->nmd->fd,
		},
		{
			.fd = nd_vale->pipe[0],
			.events = POLLIN,
		},
	};
	int ret;

	if (nd_vale->poll_rx)
		pfds[0].events |= POLLIN|POLLPRI;
	if (nd_vale->poll_tx)
		pfds[0].events |= POLLOUT;

	do {
		ret = poll(pfds, 2, -1);
	} while (ret == -1 && errno == EINTR);

	if (ret < 0) {
		perror("virtio net vale poll");
		return 0;
	}

	if (pfds[1].revents & (POLLHUP|POLLNVAL))
		return LKL_DEV_NET_POLL_HUP;

	if (pfds[1].revents & POLLIN) {
		char tmp[PIPE_BUF];

		ret = read(nd_vale->pipe[0], tmp, PIPE_BUF);
		if (ret == 0)
			return LKL_DEV_NET_POLL_HUP;
		if (ret < 0)
			perror("virtio net fd pipe read");
	}

	ret = 0;

	if (pfds[0].revents & (POLLIN|POLLPRI)) {
		nd_vale->poll_rx = 0;
		ret |= LKL_DEV_NET_POLL_RX;
	}

	if (pfds[0].revents & POLLOUT) {
		nd_vale->poll_tx = 0;
		ret |= LKL_DEV_NET_POLL_TX;
	}

	return ret;
}

static void vale_net_poll_hup(struct lkl_netdev *nd)
{
	struct lkl_netdev_vale *nd_vale =
		container_of(nd, struct lkl_netdev_vale, dev);

	/* this will cause a POLLHUP / POLLNVAL in the poll function */
	close(nd_vale->pipe[0]);
	close(nd_vale->pipe[1]);
}

static void vale_net_free(struct lkl_netdev *nd)
{
	struct lkl_netdev_vale *nd_vale =
		container_of(nd, struct lkl_netdev_vale, dev);

	nm_close(nd_vale->nmd);
	free(nd_vale);
}

struct lkl_dev_net_ops vale_net_ops =  {
	.tx = vale_net_tx,
	.rx = vale_net_rx,
	.poll = vale_net_poll,
	.poll_hup = vale_net_poll_hup,
	.free = vale_net_free,
};


struct lkl_netdev *lkl_register_netdev_vale(struct nm_desc *nmd)
{
	struct lkl_netdev_vale *nd;

	nd = malloc(sizeof(*nd));
	if (!nd) {
		fprintf(stderr, "fdnet: failed to allocate memory\n");
		/* TODO: propagate the error state, maybe use errno for that? */
		return NULL;
	}

	memset(nd, 0, sizeof(*nd));

	nd->nmd = nmd;
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

	nd->dev.ops = &vale_net_ops;
	return &nd->dev;
}

struct lkl_netdev *lkl_netdev_vale_create(char *ifname)
{
	struct lkl_netdev *nd;

	struct nm_desc *nmd;

	fprintf(stderr, "nm_open %s\n", ifname);
	nmd = nm_open(ifname, NULL, 0, 0);
	if (nmd == NULL) {
		fprintf(stderr, "open of netmap device %s failed\n", ifname);
		return NULL;
	}

	nd = lkl_register_netdev_vale(nmd);
	if (!nd) {
		perror("failed to register to.");
		nm_close(nmd);
		return NULL;
	}

	return nd;
}
