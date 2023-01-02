	.file	"test.cpp"
	.text
	.local	_ZZ4mainE1x
	.comm	_ZZ4mainE1x,4,4
	.local	_ZGVZ4mainE1x
	.comm	_ZGVZ4mainE1x,8,8
	.section	.rodata
.LC0:
	.string	"%d %d\n"
	.text
	.globl	main
	.type	main, @function
main:
.LFB0:
	.cfi_startproc
	pushq	%rbp
	.cfi_def_cfa_offset 16
	.cfi_offset 6, -16
	movq	%rsp, %rbp
	.cfi_def_cfa_register 6
	subq	$16, %rsp
	movl	$0, -4(%rbp)
.L3:
	movzbl	_ZGVZ4mainE1x(%rip), %eax
	testb	%al, %al
	sete	%al
	testb	%al, %al
	je	.L2
	leaq	_ZGVZ4mainE1x(%rip), %rdi
	call	__cxa_guard_acquire@PLT
	testl	%eax, %eax
	setne	%al
	testb	%al, %al
	je	.L2
	movl	-4(%rbp), %eax
	movl	%eax, _ZZ4mainE1x(%rip)
	leaq	_ZGVZ4mainE1x(%rip), %rdi
	call	__cxa_guard_release@PLT
.L2:
	movl	_ZZ4mainE1x(%rip), %eax
	movl	-4(%rbp), %edx
	movl	%eax, %esi
	leaq	.LC0(%rip), %rdi
	movl	$0, %eax
	call	printf@PLT
	addl	$1, -4(%rbp)
	jmp	.L3
	.cfi_endproc
.LFE0:
	.size	main, .-main
	.ident	"GCC: (Ubuntu 7.5.0-3ubuntu1~18.04) 7.5.0"
	.section	.note.GNU-stack,"",@progbits
