#include <linux/types.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>

#define MODULE_LIST_CONF "/preload_module"
#define CONFIG_SIZE (1024 * 4)

static int __init load_module(const char *filename, const char *options)
{
	int fd, ret;

	fd = sys_open(filename, O_RDONLY, 0);
	if (fd < 0)
		return fd;
	ret = sys_finit_module(fd, options ? options : "", 0);
	sys_close(fd);
	return ret;
}

static int __init find_pos(char *s, char c)
{
	char *t;

	t = s;
	while (true) {
		if (*t == '\0')
			break;
		if (*t == c)
			break;
		t++;
	}
	return t-s;
}

static int __init load_preload_modules(void)
{
	int fd, count, pos0, pos1;
	char *cache, *line, *name, *options;
	char *temp1, *temp;

	temp1 = kmalloc(CONFIG_SIZE, GFP_KERNEL);
	if (!temp1)
		return -ENOMEM;
	fd = sys_open(MODULE_LIST_CONF, O_RDONLY, 0);
	if (fd < 0) {
		kfree(temp1);
		return fd;
	}
	count = sys_read(fd, temp1, CONFIG_SIZE);
	cache = kstrndup(temp1, count, GFP_KERNEL);
	kfree(temp1);
	sys_close(fd);
	if (!cache)
		return -ENOMEM;

	temp = cache;
	for (;;) {
		if (cache + count <= temp)
			break;
		pos0 = find_pos(temp, '\n');
		if (pos0 > 0) {
			line = kstrndup(temp, pos0, GFP_KERNEL);
			temp = temp + pos0 + 1;
		} else {
			temp++;
			continue;
		}
		if (!line)
			continue;
		pos1 = find_pos(line, ' ');
		name = kstrndup(line, pos1, GFP_KERNEL);
		if (!name)
			continue;
		if (strlen(line) > pos1)
			options = line + pos1 + 1;
		else
			options = NULL;
		load_module(name, options);
		kfree(line);
		kfree(name);
	}
	kfree(cache);
	return 0;
}
