ENTRY(_start)
MEMORY { ram (rwx) : ORIGIN = 0, LENGTH = 4096 }
SECTIONS {
    .text : { KEEP(*(.text.start)) *(.text*) *(.rodata*) } > ram
    /* Include zero-initialized storage in the checked flash image, removing
       the startup clearing loop. The loader writes every initialized byte. */
    .data : { *(.data*) *(.sdata*) _bss_start = .;
              *(.bss*) *(.sbss*) *(COMMON) . = ALIGN(4); _local_data_end = .; } > ram
    .mailbox 3912 : { KEEP(*(.mailbox)) _bss_end = .; } > ram
    _image_end = .;
    __global_pointer$ = 2032;
    _stack_top = ORIGIN(ram) + LENGTH(ram);
    /* A real NOLOAD region makes the overlap check use the final relaxed
       RISC-V layout. An ASSERT can run before relaxation shrinks the code. */
    .stack (_stack_top - 128) (NOLOAD) : { . += 128; } > ram
    /DISCARD/ : { *(.comment) *(.eh_frame*) *(.riscv.attributes) }
}
