#include "selftests/i915_random.h"

static void check_scatterlist(struct drm_i915_gem_object *obj)
{
	struct scatterlist *sg;
	u64 length = 0;

	for (sg = obj->mm.pages->sgl; sg; sg = __sg_next(sg)) {
		GEM_BUG_ON(!sg_page(sg));
		length += sg->length;
	}

	GEM_BUG_ON(length != obj->base.size);
}

static int igt_shmem_clear(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct drm_i915_gem_object *obj;
	I915_RND_STATE(prng);
	void *map;
	int pfn;
	int err;

	obj = i915_gem_object_create_shmem(i915, SZ_16M);
	if (IS_ERR(obj))
		return PTR_ERR(obj);

	obj->flags |= I915_BO_CPU_CLEAR;

	map = i915_gem_object_pin_map_unlocked(obj, I915_MAP_WC);
	if (IS_ERR(map)) {
		err = PTR_ERR(obj);
		goto out;
	}
	check_scatterlist(obj);

	err = 0;
	for (pfn = 0; pfn < obj->base.size >> PAGE_SHIFT; pfn++) {
		u32 x;

		x = igt_random_offset(&prng, 0,
				      PAGE_SIZE, sizeof(x),
				      sizeof(x));
		memcpy(&x, map + x, sizeof(x));
		if (x) {
			pr_err("Found non-clear:%08x page, offset:%d\n",
			       x, pfn);
			err = -EINVAL;
			break;
		}
	}

	i915_gem_object_unpin_map(obj);
out:
	i915_gem_object_put(obj);
	return err;
}

static int __igt_shmem_swap(struct drm_i915_private *i915, bool do_swap)
{
	struct drm_i915_gem_object *obj;
	int err = 0;
	void *map;

	obj = i915_gem_object_create_shmem(i915, SZ_16M);
	if (IS_ERR(obj))
		return PTR_ERR(obj);

	i915_gem_object_lock(obj, NULL);

	map = i915_gem_object_pin_map(obj, I915_MAP_WB);
	if (IS_ERR(map)) {
		err = PTR_ERR(obj);
		goto out;
	}
	check_scatterlist(obj);

	memset(map, 0xc5, obj->base.size);
	i915_gem_object_unpin_map(obj);

	if (do_swap) {
		err = __i915_gem_object_put_pages(obj);
		if (err)
			goto out;
	}

	map = i915_gem_object_pin_map(obj, I915_MAP_WB);
	if (IS_ERR(map)) {
		err = PTR_ERR(obj);
		goto out;
	}
	check_scatterlist(obj);

	map = memchr_inv(map, 0xc5, obj->base.size);
	if (map) {
		u32 x;

		memcpy(&x, map, sizeof(x));
		pr_err("Found incorrect value:%08x at %ld\n",
		       x, map - obj->mm.mapping);
		err = -EINVAL;
	}
	i915_gem_object_unpin_map(obj);

out:
	i915_gem_object_unlock(obj);
	i915_gem_object_put(obj);
	return err;
}

static int igt_shmem_fill(void *arg)
{
	return __igt_shmem_swap(arg, false);
}

static int igt_shmem_swap(void *arg)
{
	return __igt_shmem_swap(arg, true);
}

int i915_gem_shmem_live_selftests(struct drm_i915_private *i915)
{
	static const struct i915_subtest tests[] = {
		SUBTEST(igt_shmem_clear),
		SUBTEST(igt_shmem_fill),
		SUBTEST(igt_shmem_swap),
	};

	return i915_live_subtests(tests, i915);
}
