#include <kunit/test.h>

#include <linux/mman.h>
#include <linux/random.h>

#include <linux/sched/mm.h>

unsigned long i915_vfio_test_copy(void *from, const void *to, unsigned long n)
{
	memcpy(from, to, n);

	return 0;
}

struct i915_vfio_pci_data_test {
	struct {
		void *vaddr;
		size_t size;
	} resource;
	struct i915_vfio_pci_core_device *i915_vdev;
};

size_t
i915_vfio_test_res_size(struct pci_dev *pdev, unsigned int vfid, unsigned int tile)
{
	struct i915_vfio_pci_data_test *priv = pci_get_drvdata(pdev);

	return priv->resource.size;
}

ssize_t i915_vfio_test_res_save(struct pci_dev *pdev, unsigned int vfid, unsigned int tile,
				void *buf, size_t size)
{
	struct i915_vfio_pci_data_test *priv = pci_get_drvdata(pdev);

	memcpy(buf, priv->resource.vaddr, size);

	return size;
}

int
i915_vfio_test_res_load(struct pci_dev *pdev, unsigned int vfid, unsigned int tile,
			const void *buf, size_t size)
{
	struct i915_vfio_pci_data_test *priv = pci_get_drvdata(pdev);

	memcpy(priv->resource.vaddr, buf, size);

	return 0;
}

static const struct i915_vfio_pci_migration_pf_ops pf_test_ops = {
	.ggtt.size = i915_vfio_test_res_size,
	.ggtt.save = i915_vfio_test_res_save,
	.ggtt.load = i915_vfio_test_res_load,
};

#define I915_VFIO_TEST_RES_SIZE SZ_4K
static int i915_vfio_pci_data_test_init(struct kunit *test)
{
	struct i915_vfio_pci_core_device *i915_vdev;
	struct i915_vfio_pci_migration_file *fd;
	struct i915_vfio_pci_data_test *priv;
	struct pci_dev *pdev;
	unsigned long *resource;
	size_t resource_size = I915_VFIO_TEST_RES_SIZE;
	int i;

	priv = kunit_kzalloc(test, sizeof(*priv), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, priv);

	i915_vdev = kunit_kzalloc(test, sizeof(*i915_vdev), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, i915_vdev);

	resource = kunit_kzalloc(test, resource_size, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, resource);

	for (i = 0; i < resource_size / sizeof(*resource); i++)
		resource[i] = get_random_long();

	priv->i915_vdev = i915_vdev;
	priv->resource.vaddr = resource;
	priv->resource.size = resource_size;

	i915_vdev->lmem[0].vaddr = resource;
	i915_vdev->lmem[0].size = resource_size;

	i915_vdev->pf_ops = &pf_test_ops;

	fd = kunit_kzalloc(test, sizeof(*fd), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, fd);
	i915_vdev->fd = fd;
	INIT_LIST_HEAD(&fd->save_data);
	fd->i915_vdev = i915_vdev;
	fd->copy_from = i915_vfio_test_copy;
	fd->copy_to = i915_vfio_test_copy;

	pdev = kunit_kzalloc(test, sizeof(*pdev), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, pdev);
	i915_vdev->core_device.pdev = pdev;
	i915_vdev->pf = pdev;

	pdev->vendor = 0x1234;
	pdev->device = 0x4321;

	pci_set_drvdata(pdev, priv);

	test->priv = priv;

	return 0;
}

static void i915_vfio_pci_data_test_exit(struct kunit *test)
{
	struct i915_vfio_pci_data_test *priv = test->priv;
	struct i915_vfio_pci_core_device *i915_vdev = priv->i915_vdev;

	i915_vfio_save_data_release(i915_vdev->fd);
}

static void test_produce_consume_desc(struct kunit *test)
{
	struct i915_vfio_pci_data_test *priv = test->priv;
	struct i915_vfio_pci_core_device *i915_vdev = priv->i915_vdev;
	struct i915_vfio_pci_migration_data *data;
	size_t data_len;
	ssize_t ret;
	void *buf;

	KUNIT_ASSERT_TRUE(test, list_empty(&i915_vdev->fd->save_data));
	ret = i915_vfio_produce_desc(i915_vdev->fd);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_FALSE(test, list_empty(&i915_vdev->fd->save_data));

	data = list_first_entry_or_null(&i915_vdev->fd->save_data, typeof(*data), link);
	KUNIT_ASSERT_PTR_NE(test, data, NULL);
	data_len = data->hdr.size + sizeof(data->hdr);

	buf = kunit_kzalloc(test, data_len, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, buf);

	ret = i915_vfio_data_read(i915_vdev->fd, buf, data_len);
	KUNIT_EXPECT_EQ(test, ret, data_len);

	KUNIT_EXPECT_TRUE(test, list_empty(&i915_vdev->fd->save_data));

	ret = i915_vfio_data_write(i915_vdev->fd, buf, data_len);
	KUNIT_EXPECT_EQ(test, ret, data_len);
}

static void test_produce_res(struct kunit *test)
{
	struct i915_vfio_pci_data_test *priv = test->priv;
	struct i915_vfio_pci_core_device *i915_vdev = priv->i915_vdev;
	struct i915_vfio_pci_migration_data *data;
	struct i915_vfio_pci_migration_header hdr;
	size_t data_len;
	ssize_t ret;
	void *buf;

	ret = i915_vfio_produce_ggtt(i915_vdev->fd, 0);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_FALSE(test, list_empty(&i915_vdev->fd->save_data));

	data = list_first_entry_or_null(&i915_vdev->fd->save_data, typeof(*data), link);
	KUNIT_ASSERT_PTR_NE(test, data, NULL);
	KUNIT_EXPECT_EQ(test, data->hdr.size, priv->resource.size);

	data_len = data->hdr.size + sizeof(data->hdr);
	hdr = data->hdr;

	buf = kunit_kzalloc(test, data_len, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, buf);

	ret = i915_vfio_data_read(i915_vdev->fd, buf, data_len);
	KUNIT_ASSERT_EQ(test, ret, data_len);

	KUNIT_EXPECT_TRUE(test, list_empty(&i915_vdev->fd->save_data));

	KUNIT_EXPECT_EQ(test, memcmp(buf, &hdr, sizeof(hdr)), 0);

	KUNIT_EXPECT_EQ(test,
			memcmp(buf + sizeof(hdr), priv->resource.vaddr, priv->resource.size),
			0);
}

static void test_consume_res(struct kunit *test)
{
	struct i915_vfio_pci_data_test *priv = test->priv;
	struct i915_vfio_pci_core_device *i915_vdev = priv->i915_vdev;
	struct i915_vfio_pci_migration_data *data = &i915_vdev->fd->resume_data;
	struct i915_vfio_pci_migration_header hdr = {
		.type = I915_VFIO_DATA_GGTT,
		.tile = 0,
		.offset = 0,
		.size = priv->resource.size,
		.flags = 0,
	};
	void *buf;
	size_t data_len, data_chunk;
	ssize_t ret;

	data_len = hdr.size + sizeof(hdr);
	data_chunk = sizeof(hdr) + 16;

	buf = kunit_kzalloc(test, data_len, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, buf);

	memcpy(buf, &hdr, sizeof(hdr));

	KUNIT_EXPECT_FALSE(test, data->hdr_processed);
	ret = i915_vfio_data_write(i915_vdev->fd, buf, data_chunk);
	KUNIT_ASSERT_EQ(test, ret, data_chunk);
	KUNIT_EXPECT_TRUE(test, data->hdr_processed);

	KUNIT_EXPECT_EQ(test, memcmp(&hdr, &data->hdr, sizeof(hdr)), 0);

	ret = i915_vfio_data_write(i915_vdev->fd, buf + ret, data_len - data_chunk);
	KUNIT_ASSERT_EQ(test, ret, data_len - data_chunk);

	KUNIT_EXPECT_EQ(test,
			memcmp(buf + sizeof(hdr), priv->resource.vaddr, priv->resource.size),
			0);
}

struct invalid_device_desc_test {
	char *name;
	int expected_err;
	struct i915_vfio_data_device_desc desc;
};

static void test_invalid_device_desc(struct kunit *test)
{
	const struct invalid_device_desc_test *param = test->param_value;
	struct i915_vfio_pci_data_test *priv = test->priv;
	struct i915_vfio_pci_core_device *i915_vdev = priv->i915_vdev;
	struct i915_vfio_pci_migration_header *hdr = &i915_vdev->fd->resume_data.hdr;
	void *buf;
	int ret;

	hdr->type = I915_VFIO_DATA_DESC;
	hdr->tile = 0;
	hdr->offset = 0;
	hdr->size = sizeof(param->desc);
	hdr->flags = 0;

	buf = kunit_kzalloc(test, sizeof(*hdr) + sizeof(param->desc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, buf);

	ret = i915_vfio_consume_data(i915_vdev->fd, buf, sizeof(param->desc));
	KUNIT_EXPECT_EQ(test, ret, param->expected_err);
}

static const struct invalid_device_desc_test invalid_device_desc_tests[] = {
	{
		.name = "bad magic",
		.expected_err = -EINVAL,
		.desc = {
			.magic = 0xbad,
		},
	},
	{
		.name = "bad version",
		.expected_err = -EINVAL,
		.desc = {
			.version = 0xbad,
		},
	},
	{
		.name = "bad vendor",
		.expected_err = -EINVAL,
		.desc = {
			.vendor = 0xbad,
		},
	},
	{
		.name = "bad device",
		.expected_err = -EINVAL,
		.desc = {
			.device = 0xbad,
		},
	},
};

static void invalid_device_desc_test_name(const struct invalid_device_desc_test *t,
					  char *name)
{
	snprintf(name, KUNIT_PARAM_DESC_SIZE, "%s", t->name);
}

KUNIT_ARRAY_PARAM(invalid_device_desc, invalid_device_desc_tests, invalid_device_desc_test_name);

static struct kunit_case i915_vfio_pci_data_tests[] = {
	KUNIT_CASE(test_produce_consume_desc),
	KUNIT_CASE(test_produce_res),
	KUNIT_CASE(test_consume_res),
	KUNIT_CASE_PARAM(test_invalid_device_desc, invalid_device_desc_gen_params),
	{},
};

static struct kunit_suite i915_vfio_pci_data_test_suite = {
	.name = "i915_vfio_pci_data",
	.init = i915_vfio_pci_data_test_init,
	.exit = i915_vfio_pci_data_test_exit,
	.test_cases = i915_vfio_pci_data_tests
};

kunit_test_suite(i915_vfio_pci_data_test_suite);
