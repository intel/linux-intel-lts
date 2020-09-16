#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/acpi.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <acpi/actypes.h>

MODULE_LICENSE("GPL");

#define BUFFER_SIZE (4096*5)
#define MAX_ACPI_ARGS 16
#define MIN_METHOD_LEN 4
#define DEBUG 1
#define ERROR_LEN 50
#define MARGIN 10

struct io_buffer {
	char *buffer;
	loff_t w_pos; /* current write pos */
	loff_t out_size; /* current output size */
	unsigned long size;
	int nargs;
	union acpi_object args[MAX_ACPI_ARGS];
	char *method;
	bool error;
	char error_str[ERROR_LEN];
};

static void reset_iobuffer(struct io_buffer *iobuffer)
{
	memset(iobuffer->buffer, 0, iobuffer->size);
	memset(iobuffer->args, 0, sizeof(iobuffer->args));
	iobuffer->nargs = 0;
	iobuffer->w_pos = 0;
	iobuffer->out_size = 0;
	iobuffer->method = NULL;
}

static void clear_errror_string(struct io_buffer *iobuffer)
{
	if (!iobuffer->error)
		return;

	iobuffer->error = false;
	memset(iobuffer->error_str, 0, ERROR_LEN);
}

static struct io_buffer *acpi_mediator_alloc_iobuffer(unsigned long size)
{
	struct io_buffer *iobuffer;

	iobuffer = kzalloc(sizeof(struct io_buffer), GFP_KERNEL);
	if (!iobuffer)
		return ERR_PTR(-ENOMEM);

	iobuffer->buffer = vzalloc(size);
	if (!iobuffer->buffer) {
		kfree(iobuffer);
		return ERR_PTR(-ENOMEM);
	}
	iobuffer->size = size;

	return iobuffer;
}

static void acpi_mediator_free_iobuffer(struct io_buffer *iobuffer)
{
	vfree(iobuffer->buffer);
	kfree(iobuffer);
}

static int acpi_mediator_fops_open(struct inode *inode, struct file *filep)
{

	struct io_buffer *iobuffer = acpi_mediator_alloc_iobuffer(BUFFER_SIZE);

	if (IS_ERR_OR_NULL(iobuffer))
		return -ENOMEM;

	filep->private_data = iobuffer;

	stream_open(inode, filep);

	return 0;
}
static int acpi_mediator_fops_release(struct inode *inode, struct file *filep)
{
	acpi_mediator_free_iobuffer(filep->private_data);
	return 0;
}

static int parse_acpi_args(struct io_buffer *iobuffer)
{
	unsigned long p_pos, w_pos;
	char *error = NULL;
	uint32_t arg_cnt;
	uint32_t method_type, method_size;
	uint32_t i;

	p_pos = 0;
	w_pos = iobuffer->w_pos;

#define GET_INT_INPUT(field, buf, pos, limit)			\
	do {							\
		if (pos > limit ||				\
			pos + sizeof(uint32_t) >= limit) {	\
			pr_err("wrong arg size, pos=%ld", pos);	\
			goto done;				\
		}						\
		field = *((uint32_t *)((void *)buf + pos));	\
		(pos) += sizeof(unsigned int);			\
	} while (0)

	GET_INT_INPUT(arg_cnt, iobuffer->buffer, p_pos, w_pos);

	if (arg_cnt < 1) {
		pr_err("no method name");
		goto done;
	}

	/* the first arg is method name*/
	GET_INT_INPUT(method_type, iobuffer->buffer, p_pos, w_pos);
	GET_INT_INPUT(method_size, iobuffer->buffer, p_pos, w_pos);

	if (method_type != ACPI_TYPE_STRING || method_size < MIN_METHOD_LEN) {
		pr_err("method name too short or not string type, method_type=%d, method_size=%d\n",
		       method_type, method_size);
		goto done;
	}

	if (p_pos + method_size > iobuffer->w_pos) {
		pr_err("wrong method size p_pos=%ld, method_size=%d, limit=%lld\n",
		       p_pos, method_size, iobuffer->w_pos);
		goto done;
	}
	iobuffer->method = iobuffer->buffer + p_pos;
	p_pos += method_size;

	iobuffer->nargs = arg_cnt - 1;
	/* parsing real args*/
	for (i = 0; i < iobuffer->nargs; i++) {
		uint32_t arg_type, arg_size;
		union acpi_object *arg;

		arg = &iobuffer->args[i];
		GET_INT_INPUT(arg_type, iobuffer->buffer, p_pos, w_pos);
		GET_INT_INPUT(arg_size, iobuffer->buffer, p_pos, w_pos);

		if (p_pos + arg_size > iobuffer->w_pos) {
			pr_err("wrong argument size %d, pos=%ld, limit=%lld\n",
			       arg_size, p_pos, iobuffer->w_pos);
			goto done;
		}

		arg->type = arg_type;
		switch (arg_type) {
		case ACPI_TYPE_STRING:
			arg->string.length = arg_size;
			arg->string.pointer = iobuffer->buffer + p_pos;
			break;
		case ACPI_TYPE_INTEGER:
			arg->integer.value =
				*(unsigned int *)(iobuffer->buffer + p_pos);
			break;
		case ACPI_TYPE_BUFFER:
			arg->buffer.length = arg_size;
			arg->buffer.pointer = iobuffer->buffer + p_pos;
			break;
		default:
			pr_err("not supported arg type %d\n", arg_type);
			goto done;
		}
		p_pos += arg_size;
	}

	if (iobuffer->method[method_size] != '\0')
		iobuffer->method[++method_size] = '\0';

done:
	if (error) {
		iobuffer->error = true;
		return -EINVAL;
	}
	return 0;
}

static int out_to_iobuffer(union acpi_object *out, struct io_buffer *iobuffer)
{
	unsigned long pos = 0;
	int len;
	char *error = NULL;

	memset(iobuffer->buffer, 0, iobuffer->size);
#define PUT_INT_OUTPUT(buf, pos, limit, val)			\
	do {							\
		if (pos > limit ||				\
			pos + sizeof(uint32_t) >= limit) {	\
			pr_err("wrong output size, pos=%ld\n", pos);\
			goto done;				\
		}						\
		*((uint32_t *)((void *)buf + pos)) = val;	\
		(pos) += sizeof(unsigned int);			\
	} while (0)

	if (!out)
		goto done;

	PUT_INT_OUTPUT(iobuffer->buffer, pos, iobuffer->size, 1);
	PUT_INT_OUTPUT(iobuffer->buffer, pos, iobuffer->size, out->type);

	switch (out->type) {
	case ACPI_TYPE_INTEGER:
		PUT_INT_OUTPUT(iobuffer->buffer, pos,
			       iobuffer->size, sizeof(uint32_t));
		PUT_INT_OUTPUT(iobuffer->buffer, pos,
			       iobuffer->size, out->integer.value);
		break;
	case ACPI_TYPE_STRING:
		if (pos + out->string.length > iobuffer->size)
			len = iobuffer->size - pos;
		else
			len = out->string.length;

		PUT_INT_OUTPUT(iobuffer->buffer, pos, iobuffer->size, len);
		memcpy(iobuffer->buffer + pos, out->string.pointer, len);
		pos += len;
		break;
	case ACPI_TYPE_BUFFER:
		if (pos + out->buffer.length > iobuffer->size)
			len = iobuffer->size - pos;
		else
			len = out->buffer.length;

		PUT_INT_OUTPUT(iobuffer->buffer, pos, iobuffer->size, len);
		memcpy(iobuffer->buffer + pos, out->buffer.pointer, len);
		pos += len;
		break;
	default:
		error = "unsupported output type";
		goto done;
	}
	iobuffer->out_size = pos;
done:
	if (error) {
		iobuffer->error = true;
		return -EINVAL;
	}
	return 0;
}

static int execute_acpi_method(struct io_buffer *iobuffer)
{
	struct acpi_buffer out = { ACPI_ALLOCATE_BUFFER, NULL };
	struct acpi_object_list arg;
	acpi_status status;
	acpi_handle handle;

	status = acpi_get_handle(NULL, (acpi_string) iobuffer->method, &handle);

	if (ACPI_FAILURE(status)) {
		iobuffer->error = true;
		snprintf(iobuffer->buffer, iobuffer->size, "Error: %s\n",
			 acpi_format_exception(status));
		return -EFAULT;
	}
	arg.count = iobuffer->nargs;
	arg.pointer = iobuffer->args;

	status = acpi_evaluate_object(handle, NULL, &arg, &out);
	if (ACPI_FAILURE(status)) {
		iobuffer->error = true;
		snprintf(iobuffer->buffer, iobuffer->size, "Error: %s\n",
			 acpi_format_exception(status));
		return status;
	}

	out_to_iobuffer(out.pointer, iobuffer);
	kfree(out.pointer);
	return 0;
}

/**
 * every time a write to the acpi_mediator will cause file position move forward
 */
static ssize_t acpi_mediator_fops_write(struct file *filep,
					const char __user *buf,
					size_t count, loff_t *ppos)
{
	ssize_t ret = -EINVAL;
	struct io_buffer *iobuffer = filep->private_data;
	unsigned long size = iobuffer->size;

	clear_errror_string(iobuffer);
	reset_iobuffer(iobuffer);

	if (count == 0)
		return -EINVAL;

	if (iobuffer->w_pos + count > size - MARGIN)
		count = size - MARGIN - iobuffer->w_pos;

	if (copy_from_user(iobuffer->buffer + iobuffer->w_pos, buf, count))
		return -EFAULT;

	iobuffer->w_pos += count;

	ret = parse_acpi_args(iobuffer);
	if (!ret)
		ret = execute_acpi_method(iobuffer);

	return ret < 0 ? ret : count;
}

static ssize_t acpi_mediator_fops_read(struct file *filep, char __user *buf,
			      size_t count, loff_t *ppos)
{
	ssize_t ret = -EINVAL;
	struct io_buffer *iobuffer = filep->private_data;
	int len;
	char *out;

	if (iobuffer->error)
		return -EFAULT;

	out = iobuffer->buffer;
	len = iobuffer->out_size;

	if (count > len)
		count = len;
	ret = copy_to_user(buf, out, count);

	reset_iobuffer(iobuffer);
	clear_errror_string(iobuffer);

	return ret < 0 ? ret : count - ret;
}

static const struct file_operations acpi_mediator_fops = {
	.owner		= THIS_MODULE,
	.open		= acpi_mediator_fops_open,
	.release	= acpi_mediator_fops_release,
	.read		= acpi_mediator_fops_read,
	.write		= acpi_mediator_fops_write,
};
static struct miscdevice acpi_mediator_dev = {
	.minor = 0,
	.name = "acpi_mediator",
	.fops = &acpi_mediator_fops,
	.nodename = "acpi_mediator",
	.mode = S_IRUGO | S_IWUGO,
};

static int __init init_acpi_mediator(void)
{
	int ret;

	ret = misc_register(&acpi_mediator_dev);
	if (ret) {
		pr_err("acpi mediator device register failed\n");
		return ret;
	}

	return 0;
}

static void __exit unload_acpi_mediator(void)
{
	misc_deregister(&acpi_mediator_dev);
}

module_init(init_acpi_mediator);
module_exit(unload_acpi_mediator);
