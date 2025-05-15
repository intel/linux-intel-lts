# Linux Intel LTS Kernel

## Description

This repository contains Intel's Linux LTS releases include Intel LTS source code releases, Preempt-rt kernel releases and CVE patch series releases.

Note: This should only be used for Intel platform feature evaluation and not for production or deployment with commercial Linux distribution.

## Branch information

### Basic branch naming rule

[1] main ==> Default branch, contains README.md and other documentations.

[2] x.xx/linux ==> upstream x.xx LTS kernel + Intel patches, example: 6.12/linux

[3] x.xx/linux-cve ==> CVE patches in quilt form, example: 6.12/linux-cve

    Please note: for CVE patches with config changes make sure that you regenerate kernel config using "make oldconfig"

[4] x.xx/preempt-rt ==> [linux-stable-rt](https://git.kernel.org/pub/scm/linux/kernel/git/rt/linux-stable-rt.git/) kernel + Intel patches, example: 6.6/preempt-rt. There is no separate preempt-rt kernel branch starting from 6.12.

[5] x.xx/android_t ==> upstream x.xx LTS kernel + Intel patches + Android T Dessert specific patches, example: 5.15/android_t. Same naming rule also apply to other LTS kernel version and other Android dessert.

[6] x.xx/andoird_t-cve ==> Fix CVE patches detected on x.xx/android_t branch. Same naming rule also apply to other LTS kernel version and other Android dessert.

### Active branch list - Monthly update ** 2025 May **

| Kernel Version | Active Branches |
|---------------:|-----------------|
| v6.12          | [6.12/linux](https://github.com/intel/linux-intel-lts/tree/6.12/linux) |
| v6.12          | [6.12/linux-cve](https://github.com/intel/linux-intel-lts/tree/6.12/linux-cve) |
| v6.12          | [6.12/emt](https://github.com/intel/linux-intel-lts/tree/6.12/emt) |
| v6.12          | [6.12/emt-cve](https://github.com/intel/linux-intel-lts/tree/6.12/emt-cve) |
| v6.12          | [6.12/dovetail-xenomai](https://github.com/intel/linux-intel-lts/tree/6.12/dovetail-xenomai) |
| v5.15          | [5.15/android_t](https://github.com/intel/linux-intel-lts/tree/5.15/android_t) |
| v5.15          | [5.15/android_t-cve](https://github.com/intel/linux-intel-lts/tree/5.15/android_t-cve) |
| v5.15          | [5.15/android_u](https://github.com/intel/linux-intel-lts/tree/5.15/android_u) |
| v5.15          | [5.15/android_u-cve](https://github.com/intel/linux-intel-lts/tree/5.15/android_u-cve) |

## GPG Signed Releases

### Check if a release tag is GPG-signed or not

if a tag is not signed, when you run ‘git tag -v <tag>’ command, you get the result as:

$ git tag -v lts-v4.19.272-android_t-230316T041640Z<br>
object 7150c8b4efa2baf0bef3a3da3850d29715c6fcbb<br>
type commit<br>
tag lts-v4.19.272-android_t-230316T041640Z<br>
tagger sys_oak sys_oak@intel.com 1679296599 -0700<br>

release Kernel 4.19 for android T Dessert<br>
error: no signature found<br>

You can see ‘error: no signature found’ if the tag is not signed

If the tag is signed - please follow the below steps to get the public key and verify the tag -

### Download public key

Open https://keys.openpgp.org/, input Full Key ID (i.e., EB4D99E5113E284368955757F18D9D84E60D69E7), or,
short Key ID (i.e., F18D9D84E60D69E7, the Last 16 digitals). or, the tagger email address(i.e., sys_oak@intel.com),
Click ‘Search’, then you can download the pub key file (i.e., EB4D99E5113E284368955757F18D9D84E60D69E7.asc).
The md5sum checksum is 40b0222665a5f6c70ca9d990b4014f43 for the pub key file:<br>
$ md5sum EB4D99E5113E284368955757F18D9D84E60D69E7.asc<br>
40b0222665a5f6c70ca9d990b4014f43  EB4D99E5113E284368955757F18D9D84E60D69E7.asc

Once your checksum is correct, please do next step.

### Configure your Linux Environment and verify the GPG signature of a tag ( one time setup)

After you get the right pub key, please import it:<br>
$ gpg --import EB4D99E5113E284368955757F18D9D84E60D69E7.asc

Now, when you check the tag GPG signature, you can see ‘Good signature’ with a WARNING:<br>
$ git tag -v lts-v4.19.282-android_t-230509T073627Z<br>
object 180df1199944ebd8928f320a1bd16c8a87dba2ed<br>
type commit<br>
tag lts-v4.19.282-android_t-230509T073627Z<br>
tagger sys_oak sys_oak@intel.com 1683864457 -0700

release Kernel 4.19 for android T Dessert<br>
gpg: Signature made Fri 12 May 2023 12:07:37 AM EDT<br>
gpg:                using RSA key EB4D99E5113E284368955757F18D9D84E60D69E7<br>
gpg: Good signature from "sys_oak (NSWE) sys_oak@intel.com" [unknown]<br>
gpg: WARNING: This key is not certified with a trusted signature!<br>
gpg:          There is no indication that the signature belongs to the owner.<br>
Primary key fingerprint: EB4D 99E5 113E 2843 6895  5757 F18D 9D84 E60D 69E7

To deal with the WARNING, let the pub key be trusted, run ‘gpg --edit-key <key>’ to edit it ( one time setup)<br>
$ gpg --edit-key F18D9D84E60D69E7  
input trust<br>
input 5<br>
input y<br>
input quit

Now, when you check the tag GPG signature again, you can see ‘Good signature’ without warnings:<br>
$ git tag -v lts-v4.19.282-android_t-230509T073627Z<br>
object 180df1199944ebd8928f320a1bd16c8a87dba2ed<br>
type commit<br>
tag lts-v4.19.282-android_t-230509T073627Z<br>
tagger sys_oak sys_oak@intel.com 1683864457 -0700

release Kernel 4.19 for android T Dessert<br>
gpg: Signature made Fri 12 May 2023 12:07:37 AM EDT<br>
gpg:                using RSA key EB4D99E5113E284368955757F18D9D84E60D69E7<br>
gpg: Good signature from "sys_oak (NSWE) sys_oak@intel.com" [ultimate]<br>

