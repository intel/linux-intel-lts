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

GPG Signed Releases
-------------------

i) Check if a release tag is GPG-signed or not

if a tag is not signed, when you run ‘git tag -v <tag>’ command, you get the result as:

$ git tag -v lts-v4.19.272-android_t-230316T041640Z
object 7150c8b4efa2baf0bef3a3da3850d29715c6fcbb
type commit
tag lts-v4.19.272-android_t-230316T041640Z
tagger sys_oak sys_oak@intel.com 1679296599 -0700

release Kernel 4.19 for android T Dessert
error: no signature found

You can see ‘error: no signature found’ if the tag is not signed

If the tag is signed - please follow the below steps to get the public key and verify the tag -

ii) Download public key

Open https://keys.openpgp.org/, input Full Key ID (i.e., EB4D99E5113E284368955757F18D9D84E60D69E7), or,
short Key ID (i.e., F18D9D84E60D69E7, the Last 16 digitals). or, the tagger email address(i.e., sys_oak@intel.com), 
Click ‘Search’, then you can download the pub key file (i.e., EB4D99E5113E284368955757F18D9D84E60D69E7.asc).
The md5sum checksum is 40b0222665a5f6c70ca9d990b4014f43 for the pub key file:
$ md5sum EB4D99E5113E284368955757F18D9D84E60D69E7.asc 
40b0222665a5f6c70ca9d990b4014f43  EB4D99E5113E284368955757F18D9D84E60D69E7.asc

Once your checksum is correct, please do next step.

iii) Configure your Linux Environment and verify the GPG signature of a tag ( one time setup) 

After you get the right pub key, please import it:
$ gpg --import EB4D99E5113E284368955757F18D9D84E60D69E7.asc

Now, when you check the tag GPG signature, you can see ‘Good signature’ with a WARNING:
$ git tag -v lts-v4.19.282-android_t-230509T073627Z
object 180df1199944ebd8928f320a1bd16c8a87dba2ed
type commit
tag lts-v4.19.282-android_t-230509T073627Z
tagger sys_oak sys_oak@intel.com 1683864457 -0700

release Kernel 4.19 for android T Dessert
gpg: Signature made Fri 12 May 2023 12:07:37 AM EDT
gpg:                using RSA key EB4D99E5113E284368955757F18D9D84E60D69E7
gpg: Good signature from "sys_oak (NSWE) sys_oak@intel.com" [unknown]
gpg: WARNING: This key is not certified with a trusted signature!
gpg:          There is no indication that the signature belongs to the owner.
Primary key fingerprint: EB4D 99E5 113E 2843 6895  5757 F18D 9D84 E60D 69E7

To deal with the WARNING, let the pub key be trusted, run ‘gpg --edit-key <key>’ to edit it ( one time setup)
$ gpg --edit-key F18D9D84E60D69E7  
input trust
input 5
input y
input quit

Now, when you check the tag GPG signature again , you can see ‘Good signature’ without warnings: 
$ git tag -v lts-v4.19.282-android_t-230509T073627Z
object 180df1199944ebd8928f320a1bd16c8a87dba2ed
type commit
tag lts-v4.19.282-android_t-230509T073627Z
tagger sys_oak sys_oak@intel.com 1683864457 -0700

release Kernel 4.19 for android T Dessert
gpg: Signature made Fri 12 May 2023 12:07:37 AM EDT
gpg:                using RSA key EB4D99E5113E284368955757F18D9D84E60D69E7
gpg: Good signature from "sys_oak (NSWE) sys_oak@intel.com" [ultimate]
