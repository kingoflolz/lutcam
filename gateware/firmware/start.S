.section .text.start,"ax",@progbits
.globl _start
_start:
    /* Hardware supplies fixed gp=2032 and reset sp=4096. The loader
       initializes data, BSS and the shared mailbox before releasing reset. */
    j main
