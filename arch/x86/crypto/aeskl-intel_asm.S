/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Implement AES algorithm using Intel AES Key Locker instructions.
 *
 * Most codes are based from AES-NI implementation, aesni-intel_asm.S
 *
 */

#include <linux/linkage.h>
#include <asm/inst.h>
#include <asm/frame.h>

.text

#define STATE1	%xmm0
#define STATE2	%xmm1
#define STATE3	%xmm2
#define STATE4	%xmm3
#define STATE5	%xmm4
#define STATE6	%xmm5
#define STATE7	%xmm6
#define STATE8	%xmm7
#define STATE	STATE1


#define BSWAP_MASK %xmm10
#define CTR	%xmm11
#define KEY	CTR
#define INC	%xmm12
#define IV	%xmm13

#ifdef __x86_64__
#define IN1	%xmm8
#define IN2	%xmm9
#define IN3	%xmm10
#define IN4	%xmm11
#define IN5	%xmm12
#define IN6	%xmm13
#define IN7	%xmm14
#define IN8	%xmm15
#define IN	IN1
#define TCTR_LOW	%r11
#else
#define IN	%xmm1
#endif

#ifdef __x86_64__
#define AREG	%rax
#define HANDLEP	%rdi
#define OUTP	%rsi
#define KLEN	%r9d
#define INP	%rdx
#define T1	%r10
#define LEN	%rcx
#define IVP	%r8
#else
#define AREG	%eax
#define HANDLEP	%edi
#define OUTP	AREG
#define KLEN	%ebx
#define INP	%edx
#define T1    %ecx
#define LEN %esi
#define IVP %ebp
#endif

#define UKEYP	OUTP
#define GF128MUL_MASK	%xmm10

/*
 * int _aeskl_setkey(struct crypto_aes_ctx *ctx, const u8 *in_key, unsigned int key_len)
 */
SYM_FUNC_START(_aeskl_setkey)
	FRAME_BEGIN
#ifndef __x86_64__
	push HANDLEP
	movl (FRAME_OFFSET+8)(%esp), HANDLEP	# ctx
	movl (FRAME_OFFSET+12)(%esp), UKEYP	# in_key
	movl (FRAME_OFFSET+16)(%esp), %edx	# key_len
#endif
	movl %edx, 480(HANDLEP)
	movdqu (UKEYP), STATE1
	mov $1, %eax
	cmp $16, %dl
	je .Lsetkey_128

	movdqu 0x10(UKEYP), STATE2
	encodekey256 %eax, %eax
	movdqu STATE4, 0x30(HANDLEP)
	jmp .Lsetkey_end
.Lsetkey_128:
	encodekey128 %eax, %eax

.Lsetkey_end:
	movdqu STATE1, (HANDLEP)
	movdqu STATE2, 0x10(HANDLEP)
	movdqu STATE3, 0x20(HANDLEP)

	xor AREG, AREG
#ifndef __x86_64__
	popl HANDLEP
#endif
	FRAME_END
	ret
SYM_FUNC_END(_aeskl_setkey)

/*
 * int _aeskl_enc(const void *ctx, u8 *dst, const u8 *src)
 */
SYM_FUNC_START(_aeskl_enc)
	FRAME_BEGIN
#ifndef __x86_64__
	pushl HANDLEP
	pushl KLEN
	movl (FRAME_OFFSET+12)(%esp), HANDLEP	# ctx
	movl (FRAME_OFFSET+16)(%esp), OUTP	# dst
	movl (FRAME_OFFSET+20)(%esp), INP	# src
#endif
	movdqu (INP), STATE
	movl 480(HANDLEP), KLEN

	cmp $16, KLEN
	je .Lenc_128
	aesenc256kl (HANDLEP), STATE
	jz .Lenc_err
	jmp .Lenc_noerr
.Lenc_128:
	aesenc128kl (HANDLEP), STATE
	jz .Lenc_err

.Lenc_noerr:
	xor AREG, AREG
	jmp .Lenc_end
.Lenc_err:
	mov $1, AREG
.Lenc_end:
	movdqu STATE, (OUTP)
#ifndef __x86_64__
	popl KLEN
	popl HANDLEP
#endif
	FRAME_END
	ret
SYM_FUNC_END(_aeskl_enc)

/*
 * int _aeskl_dec(const void *ctx, u8 *dst, const u8 *src)
 */
SYM_FUNC_START(_aeskl_dec)
	FRAME_BEGIN
#ifndef __x86_64__
	pushl HANDLEP
	pushl KLEN
	movl (FRAME_OFFSET+12)(%esp), HANDLEP	# ctx
	movl (FRAME_OFFSET+16)(%esp), OUTP	# dst
	movl (FRAME_OFFSET+20)(%esp), INP	# src
#endif
	movdqu (INP), STATE
	mov 480(HANDLEP), KLEN

	cmp $16, KLEN
	je .Ldec_128
	aesdec256kl (HANDLEP), STATE
	jz .Ldec_err
	jmp .Ldec_noerr
.Ldec_128:
	aesdec128kl (HANDLEP), STATE
	jz .Ldec_err

.Ldec_noerr:
	xor AREG, AREG
	jmp .Ldec_end
.Ldec_err:
	mov $1, AREG
.Ldec_end:
	movdqu STATE, (OUTP)
#ifndef __x86_64__
	popl KLEN
	popl HANDLEP
#endif
	FRAME_END
	ret
SYM_FUNC_END(_aeskl_dec)

/*
 * int _aeskl_ecb_enc(struct crypto_aes_ctx *ctx, const u8 *dst, u8 *src, size_t len)
 */
SYM_FUNC_START(_aeskl_ecb_enc)
	FRAME_BEGIN
#ifndef __x86_64__
	pushl LEN
	pushl HANDLEP
	pushl KLEN
	movl (FRAME_OFFSET+16)(%esp), HANDLEP	# ctx
	movl (FRAME_OFFSET+20)(%esp), OUTP	# dst
	movl (FRAME_OFFSET+24)(%esp), INP	# src
	movl (FRAME_OFFSET+28)(%esp), LEN	# len
#endif
	test LEN, LEN
	jz .Lecb_enc_noerr
	mov 480(HANDLEP), KLEN
	cmp $16, LEN
	jb .Lecb_enc_noerr
	cmp $128, LEN
	jb .Lecb_enc1

.align 4
.Lecb_enc8:
	movdqu (INP), STATE1
	movdqu 0x10(INP), STATE2
	movdqu 0x20(INP), STATE3
	movdqu 0x30(INP), STATE4
	movdqu 0x40(INP), STATE5
	movdqu 0x50(INP), STATE6
	movdqu 0x60(INP), STATE7
	movdqu 0x70(INP), STATE8

	cmp $16, KLEN
	je .Lecb_enc8_128
	aesencwide256kl (HANDLEP)
	jz .Lecb_enc_err
	jmp .Lecb_enc8_end
.Lecb_enc8_128:
	aesencwide128kl (HANDLEP)
	jz .Lecb_enc_err

.Lecb_enc8_end:
	movdqu STATE1, (OUTP)
	movdqu STATE2, 0x10(OUTP)
	movdqu STATE3, 0x20(OUTP)
	movdqu STATE4, 0x30(OUTP)
	movdqu STATE5, 0x40(OUTP)
	movdqu STATE6, 0x50(OUTP)
	movdqu STATE7, 0x60(OUTP)
	movdqu STATE8, 0x70(OUTP)

	sub $128, LEN
	add $128, INP
	add $128, OUTP
	cmp $128, LEN
	jge .Lecb_enc8
	cmp $16, LEN
	jb .Lecb_enc_noerr

.align 4
.Lecb_enc1:
	movdqu (INP), STATE1
	cmp $16, KLEN
	je .Lecb_enc1_128
	aesenc256kl (HANDLEP), STATE
	jz .Lecb_enc_err
	jmp .Lecb_enc1_end
.Lecb_enc1_128:
	aesenc128kl (HANDLEP), STATE
	jz .Lecb_enc_err

.Lecb_enc1_end:
	movdqu STATE1, (OUTP)
	sub $16, LEN
	add $16, INP
	add $16, OUTP
	cmp $16, LEN
	jge .Lecb_enc1

.Lecb_enc_noerr:
	xor AREG, AREG
	jmp .Lecb_enc_end
.Lecb_enc_err:
	mov $1, AREG
.Lecb_enc_end:
#ifndef __x86_64__
	popl KLEN
	popl HANDLEP
	popl LEN
#endif
	FRAME_END
	ret
SYM_FUNC_END(_aeskl_ecb_enc)

/*
 * int _aeskl_ecb_dec(struct crypto_aes_ctx *ctx, const u8 *dst, u8 *src, size_t len);
 */
SYM_FUNC_START(_aeskl_ecb_dec)
	FRAME_BEGIN
#ifndef __x86_64__
	pushl LEN
	pushl HANDLEP
	pushl KLEN
	movl (FRAME_OFFSET+16)(%esp), HANDLEP	# ctx
	movl (FRAME_OFFSET+20)(%esp), OUTP	# dst
	movl (FRAME_OFFSET+24)(%esp), INP	# src
	movl (FRAME_OFFSET+28)(%esp), LEN	# len
#endif

	test LEN, LEN
	jz .Lecb_dec_noerr
	mov 480(HANDLEP), KLEN
	cmp $16, LEN
	jb .Lecb_dec_noerr
	cmp $128, LEN
	jb .Lecb_dec1

.align 4
.Lecb_dec8:
	movdqu (INP), STATE1
	movdqu 0x10(INP), STATE2
	movdqu 0x20(INP), STATE3
	movdqu 0x30(INP), STATE4
	movdqu 0x40(INP), STATE5
	movdqu 0x50(INP), STATE6
	movdqu 0x60(INP), STATE7
	movdqu 0x70(INP), STATE8

	cmp $16, KLEN
	je .Lecb_dec8_128
	aesdecwide256kl (HANDLEP)
	jz .Lecb_dec_err
	jmp .Lecb_dec8_end
.Lecb_dec8_128:
	aesdecwide128kl (HANDLEP)
	jz .Lecb_dec_err

.Lecb_dec8_end:
	movdqu STATE1, (OUTP)
	movdqu STATE2, 0x10(OUTP)
	movdqu STATE3, 0x20(OUTP)
	movdqu STATE4, 0x30(OUTP)
	movdqu STATE5, 0x40(OUTP)
	movdqu STATE6, 0x50(OUTP)
	movdqu STATE7, 0x60(OUTP)
	movdqu STATE8, 0x70(OUTP)

	sub $128, LEN
	add $128, INP
	add $128, OUTP
	cmp $128, LEN
	jge .Lecb_dec8
	cmp $16, LEN
	jb .Lecb_dec_noerr

.align 4
.Lecb_dec1:
	movdqu (INP), STATE1
	cmp $16, KLEN
	je .Lecb_dec1_128
	aesdec256kl (HANDLEP), STATE
	jz .Lecb_dec_err
	jmp .Lecb_dec1_end
.Lecb_dec1_128:
	aesdec128kl (HANDLEP), STATE
	jz .Lecb_dec_err

.Lecb_dec1_end:
	movdqu STATE1, (OUTP)
	sub $16, LEN
	add $16, INP
	add $16, OUTP
	cmp $16, LEN
	jge .Lecb_dec1

.Lecb_dec_noerr:
	xor AREG, AREG
	jmp .Lecb_dec_end
.Lecb_dec_err:
	mov $1, AREG
.Lecb_dec_end:
#ifndef __x86_64__
	popl KLEN
	popl HANDLEP
	popl LEN
#endif
	FRAME_END
	ret
SYM_FUNC_END(_aeskl_ecb_dec)

/*
 * int _aeskl_cbc_enc(struct crypto_aes_ctx *ctx, const u8 *dst, u8 *src, size_t len, u8 *iv)
 */
SYM_FUNC_START(_aeskl_cbc_enc)
	FRAME_BEGIN
#ifndef __x86_64__
	pushl IVP
	pushl LEN
	pushl HANDLEP
	pushl KLEN
	movl (FRAME_OFFSET+20)(%esp), HANDLEP	# ctx
	movl (FRAME_OFFSET+24)(%esp), OUTP	# dst
	movl (FRAME_OFFSET+28)(%esp), INP	# src
	movl (FRAME_OFFSET+32)(%esp), LEN	# len
	movl (FRAME_OFFSET+36)(%esp), IVP	# iv
#endif

	cmp $16, LEN
	jb .Lcbc_enc_noerr
	mov 480(HANDLEP), KLEN
	movdqu (IVP), STATE

.align 4
.Lcbc_enc1:
	movdqu (INP), IN
	pxor IN, STATE

	cmp $16, KLEN
	je .Lcbc_enc1_128
	aesenc256kl (HANDLEP), STATE
	jz .Lcbc_enc_err
	jmp .Lcbc_enc1_end
.Lcbc_enc1_128:
	aesenc128kl (HANDLEP), STATE
	jz .Lcbc_enc_err

.Lcbc_enc1_end:
	movdqu STATE, (OUTP)
	sub $16, LEN
	add $16, INP
	add $16, OUTP
	cmp $16, LEN
	jge .Lcbc_enc1
	movdqu STATE, (IVP)

.Lcbc_enc_noerr:
	xor AREG, AREG
	jmp .Lcbc_enc_end
.Lcbc_enc_err:
	mov $1, AREG
.Lcbc_enc_end:
#ifndef __x86_64__
	popl KLEN
	popl HANDLEP
	popl LEN
	popl IVP
#endif
	FRAME_END
	ret
SYM_FUNC_END(_aeskl_cbc_enc)

/*
 * int _aeskl_cbc_dec(struct crypto_aes_ctx *ctx, const u8 *dst, u8 *src, size_t len, u8 *iv)
 */
SYM_FUNC_START(_aeskl_cbc_dec)
	FRAME_BEGIN
#ifndef __x86_64__
	pushl IVP
	pushl LEN
	pushl HANDLEP
	pushl KLEN
	movl (FRAME_OFFSET+20)(%esp), HANDLEP	# ctx
	movl (FRAME_OFFSET+24)(%esp), OUTP	# dst
	movl (FRAME_OFFSET+28)(%esp), INP	# src
	movl (FRAME_OFFSET+32)(%esp), LEN	# len
	movl (FRAME_OFFSET+36)(%esp), IVP	# iv
#endif

	cmp $16, LEN
	jb .Lcbc_dec_noerr
	mov 480(HANDLEP), KLEN
#ifdef __x86_64__
	cmp $128, LEN
	jb .Lcbc_dec1_pre

.align 4
.Lcbc_dec8:
	movdqu 0x0(INP), STATE1
	movdqu 0x10(INP), STATE2
	movdqu 0x20(INP), STATE3
	movdqu 0x30(INP), STATE4
	movdqu 0x40(INP), STATE5
	movdqu 0x50(INP), STATE6
	movdqu 0x60(INP), STATE7
	movdqu 0x70(INP), STATE8

	movdqu (IVP), IN1
	movdqa STATE1, IN2
	movdqa STATE2, IN3
	movdqa STATE3, IN4
	movdqa STATE4, IN5
	movdqa STATE5, IN6
	movdqa STATE6, IN7
	movdqa STATE7, IN8
	movdqu STATE8, (IVP)

	cmp $16, KLEN
	je .Lcbc_dec8_128
	aesdecwide256kl (HANDLEP)
	jz .Lcbc_dec_err
	jmp .Lcbc_dec8_end
.Lcbc_dec8_128:
	aesdecwide128kl (HANDLEP)
	jz .Lcbc_dec_err

.Lcbc_dec8_end:
	pxor IN1, STATE1
	pxor IN2, STATE2
	pxor IN3, STATE3
	pxor IN4, STATE4
	pxor IN5, STATE5
	pxor IN6, STATE6
	pxor IN7, STATE7
	pxor IN8, STATE8

	movdqu STATE1, 0x0(OUTP)
	movdqu STATE2, 0x10(OUTP)
	movdqu STATE3, 0x20(OUTP)
	movdqu STATE4, 0x30(OUTP)
	movdqu STATE5, 0x40(OUTP)
	movdqu STATE6, 0x50(OUTP)
	movdqu STATE7, 0x60(OUTP)
	movdqu STATE8, 0x70(OUTP)

	sub $128, LEN
	add $128, INP
	add $128, OUTP
	cmp $128, LEN
	jge .Lcbc_dec8
	cmp $16, LEN
	jb .Lcbc_dec_noerr
#endif

.align 4
.Lcbc_dec1_pre:
	movdqu (IVP), STATE3
.Lcbc_dec1:
	movdqu (INP), STATE2
	movdqa STATE2, STATE1

	cmp $16, KLEN
	je .Lcbc_dec1_128
	aesdec256kl (HANDLEP), STATE1
	jz .Lcbc_dec_err
	jmp .Lcbc_dec1_end
.Lcbc_dec1_128:
	aesdec128kl (HANDLEP), STATE1
	jz .Lcbc_dec_err

.Lcbc_dec1_end:
	pxor STATE3, STATE1
	movdqu STATE1, (OUTP)
	movdqa STATE2, STATE3
	sub $16, LEN
	add $16, INP
	add $16, OUTP
	cmp $16, LEN
	jge .Lcbc_dec1
	movdqu STATE3, (IVP)

.Lcbc_dec_noerr:
	xor AREG, AREG
	jmp .Lcbc_dec_end
.Lcbc_dec_err:
	mov $1, AREG
.Lcbc_dec_end:
#ifndef __x86_64__
	popl KLEN
	popl HANDLEP
	popl LEN
	popl IVP
#endif
	FRAME_END
	ret
SYM_FUNC_END(_aeskl_cbc_dec)

/*
 * XTS implementation
 */

.section	.rodata.cst16.gf128mul_x_ble_mask, "aM", @progbits, 16
.align 16
.Lgf128mul_x_ble_mask:
	.octa 0x00000000000000010000000000000087
.previous

/*
 * _aes_gf128mul_x_ble:		internal ABI
 *	Multiply in GF(2^128) for XTS IVs
 * input:
 *	IV:	current IV
 *	GF128MUL_MASK == mask with 0x87 and 0x01
 * output:
 *	IV:	next IV
 * changed:
 *	CTR:	== temporary value
 */
#define _aes_gf128mul_x_ble() \
	pshufd $0x13, IV, KEY; \
	paddq IV, IV; \
	psrad $31, KEY; \
	pand GF128MUL_MASK, KEY; \
	pxor KEY, IV;

/*
 * int _aeskl_xts_crypt8(const struct crypto_aes_ctx *ctx,
 *			 const u8 *dst,
 *			 u8 *src,
 *			 bool enc,
 *			 u8 *iv)
 */
SYM_FUNC_START(_aeskl_xts_crypt8)
	FRAME_BEGIN

	movdqa .Lgf128mul_x_ble_mask, GF128MUL_MASK
	movdqu (IVP), IV

	mov 480(HANDLEP), KLEN

	movdqa IV, STATE1
	movdqu (INP), INC
	pxor INC, STATE1
	movdqu IV, (OUTP)

	_aes_gf128mul_x_ble()
	movdqa IV, STATE2
	movdqu 0x10(INP), INC
	pxor INC, STATE2
	movdqu IV, 0x10(OUTP)

	_aes_gf128mul_x_ble()
	movdqa IV, STATE3
	movdqu 0x20(INP), INC
	pxor INC, STATE3
	movdqu IV, 0x20(OUTP)

	_aes_gf128mul_x_ble()
	movdqa IV, STATE4
	movdqu 0x30(INP), INC
	pxor INC, STATE4
	movdqu IV, 0x30(OUTP)

	_aes_gf128mul_x_ble()
	movdqa IV, STATE5
	movdqu 0x40(INP), INC
	pxor INC, STATE5
	movdqu IV, 0x40(OUTP)

	_aes_gf128mul_x_ble()
	movdqa IV, STATE6
	movdqu 0x50(INP), INC
	pxor INC, STATE6
	movdqu IV, 0x50(OUTP)

	_aes_gf128mul_x_ble()
	movdqa IV, STATE7
	movdqu 0x60(INP), INC
	pxor INC, STATE7
	movdqu IV, 0x60(OUTP)

	_aes_gf128mul_x_ble()
	movdqa IV, STATE8
	movdqu 0x70(INP), INC
	pxor INC, STATE8
	movdqu IV, 0x70(OUTP)

	cmpb $0, %cl
	je  .Lxts_dec8
	cmp $16, KLEN
	je .Lxts_enc8_128
	aesencwide256kl (%rdi)
	jz .Lxts_err
	jmp .Lxts_crypt8_end
.Lxts_enc8_128:
	aesencwide128kl (%rdi)
	jz .Lxts_err
	jmp .Lxts_crypt8_end
.Lxts_dec8:
	cmp $16, KLEN
	je .Lxts_dec8_128
	aesdecwide256kl (%rdi)
	jz .Lxts_err
	jmp .Lxts_crypt8_end
.Lxts_dec8_128:
	aesdecwide128kl (%rdi)
	jz .Lxts_err

.Lxts_crypt8_end:
	movdqu 0x00(OUTP), INC
	pxor INC, STATE1
	movdqu STATE1, 0x00(OUTP)

	movdqu 0x10(OUTP), INC
	pxor INC, STATE2
	movdqu STATE2, 0x10(OUTP)

	movdqu 0x20(OUTP), INC
	pxor INC, STATE3
	movdqu STATE3, 0x20(OUTP)

	movdqu 0x30(OUTP), INC
	pxor INC, STATE4
	movdqu STATE4, 0x30(OUTP)

	movdqu 0x40(OUTP), INC
	pxor INC, STATE5
	movdqu STATE5, 0x40(OUTP)

	movdqu 0x50(OUTP), INC
	pxor INC, STATE6
	movdqu STATE6, 0x50(OUTP)

	movdqu 0x60(OUTP), INC
	pxor INC, STATE7
	movdqu STATE7, 0x60(OUTP)

	movdqu 0x70(OUTP), INC
	pxor INC, STATE8
	movdqu STATE8, 0x70(OUTP)

	_aes_gf128mul_x_ble()
	movdqu IV, (IVP)

	xor AREG, AREG
	jmp .Lxts_end
.Lxts_err:
	mov $1, AREG
.Lxts_end:
	FRAME_END
	ret
SYM_FUNC_END(_aeskl_xts_crypt8)

#ifdef __x86_64__

.pushsection .rodata
.align 16
.Lcts_permute_table:
	.byte		0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80
	.byte		0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80
	.byte		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
	.byte		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
	.byte		0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80
	.byte		0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80
.Lbswap_mask:
	.byte 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0
.popsection

/*
 * _aes_ctr_inc_init:	internal ABI
 *	setup registers used by _aesni_inc
 * input:
 *	IV
 * output:
 *	CTR:	== IV, in little endian
 *	TCTR_LOW: == lower qword of CTR
 *	INC:	== 1, in little endian
 *	BSWAP_MASK == endian swapping mask
 */
SYM_FUNC_START_LOCAL(_aes_ctr_inc_init)
	movaps .Lbswap_mask, BSWAP_MASK
	movaps IV, CTR
	pshufb BSWAP_MASK, CTR
	mov $1, TCTR_LOW
	movq TCTR_LOW, INC
	movq CTR, TCTR_LOW
	ret
SYM_FUNC_END(_aes_ctr_inc_init)

/*
 * _aes_ctr_inc:		internal ABI
 *	Increase IV by 1, IV is in big endian
 * input:
 *	IV
 *	CTR:	== IV, in little endian
 *	TCTR_LOW: == lower qword of CTR
 *	INC:	== 1, in little endian
 *	BSWAP_MASK == endian swapping mask
 * output:
 *	IV:	Increase by 1
 * changed:
 *	CTR:	== output IV, in little endian
 *	TCTR_LOW: == lower qword of CTR
 */
SYM_FUNC_START_LOCAL(_aes_ctr_inc)
	paddq INC, CTR
	add $1, TCTR_LOW
	jnc .Linc_low
	pslldq $8, INC
	paddq INC, CTR
	psrldq $8, INC
.Linc_low:
	movaps CTR, IV
	pshufb BSWAP_MASK, IV
	ret
SYM_FUNC_END(_aes_ctr_inc)

/*
 * CTR implementations
 */

/*
 * int _aeskl_ctr_enc(struct crypto_aes_ctx *ctx, const u8 *dst, u8 *src, size_t len, u8 *iv)
 */
SYM_FUNC_START(_aeskl_ctr_enc)
	FRAME_BEGIN
	cmp $16, LEN
	jb .Lctr_enc_noerr
	mov 480(HANDLEP), KLEN
	movdqu (IVP), IV
	call _aes_ctr_inc_init
	cmp $128, LEN
	jb .Lctr_enc1

.align 4
.Lctr_enc8:
	movaps IV, STATE1
	call _aes_ctr_inc
	movaps IV, STATE2
	call _aes_ctr_inc
	movaps IV, STATE3
	call _aes_ctr_inc
	movaps IV, STATE4
	call _aes_ctr_inc
	movaps IV, STATE5
	call _aes_ctr_inc
	movaps IV, STATE6
	call _aes_ctr_inc
	movaps IV, STATE7
	call _aes_ctr_inc
	movaps IV, STATE8
	call _aes_ctr_inc

	cmp $16, KLEN
	je .Lctr_enc8_128
	aesencwide256kl (%rdi)
	jz .Lctr_enc_err
	jmp .Lctr_enc8_end
.Lctr_enc8_128:
	aesencwide128kl (%rdi)
	jz .Lctr_enc_err
.Lctr_enc8_end:

	movups (INP), IN1
	pxor IN1, STATE1
	movups STATE1, (OUTP)

	movups 0x10(INP), IN1
	pxor IN1, STATE2
	movups STATE2, 0x10(OUTP)

	movups 0x20(INP), IN1
	pxor IN1, STATE3
	movups STATE3, 0x20(OUTP)

	movups 0x30(INP), IN1
	pxor IN1, STATE4
	movups STATE4, 0x30(OUTP)

	movups 0x40(INP), IN1
	pxor IN1, STATE5
	movups STATE5, 0x40(OUTP)

	movups 0x50(INP), IN1
	pxor IN1, STATE6
	movups STATE6, 0x50(OUTP)

	movups 0x60(INP), IN1
	pxor IN1, STATE7
	movups STATE7, 0x60(OUTP)

	movups 0x70(INP), IN1
	pxor IN1, STATE8
	movups STATE8, 0x70(OUTP)

	sub $128, LEN
	add $128, INP
	add $128, OUTP
	cmp $128, LEN
	jge .Lctr_enc8
	cmp $16, LEN
	jb .Lctr_enc_end

.align 4
.Lctr_enc1:
	movaps IV, STATE1
	call _aes_ctr_inc

	cmp $16, KLEN
	je .Lctr_enc1_128
	aesenc256kl (HANDLEP), STATE1
	jz .Lctr_enc_err
	jmp .Lctr_enc1_end
.Lctr_enc1_128:
	aesenc128kl (HANDLEP), STATE1
	jz .Lctr_enc_err

.Lctr_enc1_end:
	movups (INP), IN1
	pxor IN1, STATE1
	movups STATE1, (OUTP)
	sub $16, LEN
	add $16, INP
	add $16, OUTP
	cmp $16, LEN
	jge .Lctr_enc1

.Lctr_enc_end:
	movdqu IV, (IVP)
.Lctr_enc_noerr:
	xor AREG, AREG
	jmp .Lctr_enc_ret
.Lctr_enc_err:
	mov $1, AREG
.Lctr_enc_ret:
	FRAME_END
	ret
SYM_FUNC_END(_aeskl_ctr_enc)

#endif
