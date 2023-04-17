// SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
// Copyright (C) 2019 Mellanox.

#define _GNU_SOURCE
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <net/if.h>
#include <linux/if.h>
#include <linux/rtnetlink.h>
#include <sys/socket.h>

#include "bpf/nlattr.h"
#include "main.h"
#include "netlink_dumper.h"


/* TODO: reuse  form net.c */
#ifndef SOL_NETLINK
#define SOL_NETLINK 270
#endif

static int netlink_open(__u32 *nl_pid)
{
	struct sockaddr_nl sa;
	socklen_t addrlen;
	int one = 1, ret;
	int sock;

	memset(&sa, 0, sizeof(sa));
	sa.nl_family = AF_NETLINK;

	sock = socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
	if (sock < 0)
		return -errno;

	if (setsockopt(sock, SOL_NETLINK, NETLINK_EXT_ACK,
		       &one, sizeof(one)) < 0) {
		p_err("Netlink error reporting not supported");
	}

	if (bind(sock, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
		ret = -errno;
		goto cleanup;
	}

	addrlen = sizeof(sa);
	if (getsockname(sock, (struct sockaddr *)&sa, &addrlen) < 0) {
		ret = -errno;
		goto cleanup;
	}

	if (addrlen != sizeof(sa)) {
		ret = -LIBBPF_ERRNO__INTERNAL;
		goto cleanup;
	}

	*nl_pid = sa.nl_pid;
	return sock;

cleanup:
	close(sock);
	return ret;
}

typedef int (*dump_nlmsg_t)(void *cookie, void *msg, struct nlattr **tb);

typedef int (*__dump_nlmsg_t)(struct nlmsghdr *nlmsg, dump_nlmsg_t, void *cookie);

static int netlink_recv(int sock, __u32 nl_pid, __u32 seq,
			__dump_nlmsg_t _fn, dump_nlmsg_t fn,
			void *cookie)
{
	bool multipart = true;
	struct nlmsgerr *err;
	struct nlmsghdr *nh;
	char buf[4096];
	int len, ret;

	while (multipart) {
		multipart = false;
		len = recv(sock, buf, sizeof(buf), 0);
		if (len < 0) {
			ret = -errno;
			goto done;
		}

		if (len == 0)
			break;

		for (nh = (struct nlmsghdr *)buf; NLMSG_OK(nh, len);
		     nh = NLMSG_NEXT(nh, len)) {
			if (nh->nlmsg_pid != nl_pid) {
				ret = -LIBBPF_ERRNO__WRNGPID;
				goto done;
			}
			if (nh->nlmsg_seq != seq) {
				ret = -LIBBPF_ERRNO__INVSEQ;
				goto done;
			}
			if (nh->nlmsg_flags & NLM_F_MULTI)
				multipart = true;
			switch (nh->nlmsg_type) {
			case NLMSG_ERROR:
				err = (struct nlmsgerr *)NLMSG_DATA(nh);
				if (!err->error)
					continue;
				ret = err->error;
				libbpf_nla_dump_errormsg(nh);
				goto done;
			case NLMSG_DONE:
				return 0;
			default:
				break;
			}
			if (_fn) {
				ret = _fn(nh, fn, cookie);
				if (ret)
					return ret;
			}
		}
	}
	ret = 0;
done:
	return ret;
}


static int __dump_link_nlmsg(struct nlmsghdr *nlh,
			     dump_nlmsg_t dump_link_nlmsg, void *cookie)
{
	struct nlattr *tb[IFLA_MAX + 1], *attr;
	struct ifinfomsg *ifi = NLMSG_DATA(nlh);
	int len;

	len = nlh->nlmsg_len - NLMSG_LENGTH(sizeof(*ifi));
	attr = (struct nlattr *) ((void *) ifi + NLMSG_ALIGN(sizeof(*ifi)));
	if (libbpf_nla_parse(tb, IFLA_MAX, attr, len, NULL) != 0)
		return -LIBBPF_ERRNO__NLPARSE;

	return dump_link_nlmsg(cookie, ifi, tb);
}

static int netlink_get_link(int sock, unsigned int nl_pid,
			    dump_nlmsg_t dump_link_nlmsg, void *cookie)
{
	struct {
		struct nlmsghdr nlh;
		struct ifinfomsg ifm;
	} req = {
		.nlh.nlmsg_len = NLMSG_LENGTH(sizeof(struct ifinfomsg)),
		.nlh.nlmsg_type = RTM_GETLINK,
		.nlh.nlmsg_flags = NLM_F_DUMP | NLM_F_REQUEST,
		.ifm.ifi_family = AF_PACKET,
	};
	int seq = time(NULL);

	req.nlh.nlmsg_seq = seq;
	if (send(sock, &req, req.nlh.nlmsg_len, 0) < 0)
		return -errno;

	return netlink_recv(sock, nl_pid, seq, __dump_link_nlmsg,
			    dump_link_nlmsg, cookie);
}

struct ip_devname_ifindex {
	char	devname[64];
	int	ifindex;
};

struct bpf_netdev_t {
	struct ip_devname_ifindex *devices;
	int	used_len;
	int	array_len;
	int	filter_idx;
};

static int do_show(int argc, char **argv)
{
	int sock, ret, filter_idx = -1;
	struct bpf_netdev_t dev_array;
	unsigned int nl_pid = 0;
	char err_buf[256];

	if (argc == 2) {
		if (strcmp(argv[0], "dev") != 0)
			usage();
		filter_idx = if_nametoindex(argv[1]);
		if (filter_idx == 0) {
			fprintf(stderr, "invalid dev name %s\n", argv[1]);
			return -1;
		}
	} else if (argc != 0) {
		usage();
	}

	sock = netlink_open(&nl_pid);
	if (sock < 0) {
		fprintf(stderr, "failed to open netlink sock\n");
		return -1;
	}

	dev_array.devices = NULL;
	dev_array.used_len = 0;
	dev_array.array_len = 0;
	dev_array.filter_idx = filter_idx;

	if (json_output)
		jsonw_start_array(json_wtr);
	NET_START_OBJECT;
	NET_START_ARRAY("xdp", "%s:\n");
	ret = netlink_get_link(sock, nl_pid, xdp_dump_link_nlmsg, &dev_array);
	NET_END_ARRAY("\n");

	NET_END_OBJECT;
	if (json_output)
		jsonw_end_array(json_wtr);

	if (ret) {
		if (json_output)
			jsonw_null(json_wtr);
		libbpf_strerror(ret, err_buf, sizeof(err_buf));
		fprintf(stderr, "Error: %s\n", err_buf);
	}
	free(dev_array.devices);
	close(sock);
	return ret;
}

static int set_usage(void)
{
	fprintf(stderr,
		"Usage: %s net xdp set dev <devname> {md_btf {on|off}}\n"
		"       %s net xdp set help\n"
		"       md_btf {on|off}: enable/disable meta data btf\n",
		bin_name, bin_name);

	return -1;
}

static int xdp_set_md_btf(int ifindex, char *arg)
{
	__u8 enable = (strcmp(arg, "on") == 0) ? 1 : 0;
	int ret;

	ret = bpf_set_link_xdp_md_btf(ifindex, enable);
	if (ret)
		fprintf(stderr, "Failed to setup xdp md, err=%d\n", ret);

	return -ret;
}

static int do_set(int argc, char **argv)
{
	char *set_cmd, *set_arg;
	int dev_idx = -1;

	if (argc < 4)
		return set_usage();

	if (strcmp(argv[0], "dev") != 0)
		return set_usage();

	dev_idx = if_nametoindex(argv[1]);
	if (dev_idx == 0) {
		fprintf(stderr, "invalid dev name %s\n", argv[1]);
		return -1;
	}

	set_cmd = argv[2];
	set_arg = argv[3];

	if (strcmp(set_cmd, "md_btf") != 0)
		return set_usage();

	if (strcmp(set_arg, "on") != 0 && strcmp(set_arg, "off") != 0)
		return set_usage();

	return xdp_set_md_btf(dev_idx, set_arg);
}

static int do_help(int argc, char **argv)
{
	if (json_output) {
		jsonw_null(json_wtr);
		return 0;
	}

	fprintf(stderr,
		"Usage: %s %s xdp { show | list | set } [dev <devname>]\n"
		"       %s %s help\n",
		bin_name, argv[-2], bin_name, argv[-2]);

	return 0;
}

static const struct cmd cmds[] = {
	{ "show",        do_show },
	{ "list",        do_show },
	{ "set",         do_set  },
	{ "help",        do_help },
	{ 0 }
};

int do_xdp(int argc, char **argv)
{
	return cmd_select(cmds, argc, argv, do_help);
}
