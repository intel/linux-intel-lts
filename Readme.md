This repository contains Intel's Linux LTS releases. 

Branch information
-------------------

[1]  Master ==> follows kernel.org master. updated periodically

[2]  4.x/base ==> upstream 4.x LTS kernel + Intel patches (bug fixes/ optimizations)

[3]  4.x/android ==> upstream 4.x LTS kernel + Intel patches + Android P Dessert specific patches

[4]  4.x/android_q ==> upstream 4.x LTS kernel + Intel patches + Android Q Dessert specific patches

[5]  4.x/yocto ==> upstream 4.x LTS kernel with yocto specific patches

[6]  4.x/preempt-rt ==> upstream 4.x preempt rt kernel + Intel patches

[7]  4.x/base-cve, 4.x/android-cve, 4.x/android_q-cve ==> fixes for CVEs detected on 4.x stable. The patches are released in quilt form. 
     Use "git quiltimport" to apply the patches. The CVE patches are matched to kernel source using tags
     for ex. CVE patches tagged with lts-v4.19.44-base-cve-190524T175309Z or lts-v4.19.44-android_q-cve-190524T175309Z are applied on kernel source tagged with
     lts-v4.19.44-base-190524T175309Z or lts-v4.19.44-android_q-190524T175309Z 
    
     Please note: for CVE patches with config changes make sure that you regenerate kernel config using "make oldconfig"
