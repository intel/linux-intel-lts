===============
DRM Driver uAPI
===============

drm/i915 uAPI
=============

This section describes the i915's userspace API for the out-of-tree releases.

It is a combination of 2 sets of IOCTLs and structs:

1. A not yet upstreamed set called PRELIM. See :ref:`i915 PRELIM uAPI`.
2. An already upstreamed set. See :ref:`i915 UPSTREAMED uAPI`.

.. _i915 PRELIM uAPI:

drm/i915 PRELIM uAPI
--------------------

Current version: 2.0
UMDs can check the version at runtime at /sys/<...>/drm/card/prelim_uapi_version

'PRELIM' uAPIs are APIs that are not yet merged on upstream i915. They were
designed to be in a different range in a way that the divergence with the
upstream can be controlled and the conflicts minimized.

It is additional to the regular i915 uAPI (i915_drm.h).
See :ref:`i915 UPSTREAMED uAPI`.
Whenever a functionality is available in both `i915 UPSTREAMED uAPI` and this
PRELIM uAPI, please prefer the `i915 UPSTREAMED uAPI` one.

From the Linux upstream perspective they are considered preliminary.
However from the Out-of-tree releases perspective they should be treated
as final, stable, reliable and immutable.

.. kernel-doc:: include/uapi/drm/i915_drm_prelim.h

.. _i915 UPSTREAMED uAPI:

drm/i915 UPSTREAMED uAPI
------------------------

.. kernel-doc:: include/uapi/drm/i915_drm.h
