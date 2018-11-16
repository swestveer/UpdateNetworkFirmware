    ; Author: Alex Wranovsky

    ; function to call the bootloader
    .global call_bootloader
    .thumbfunc call_bootloader
    .align 4
call_bootloader: .asmfunc
    ; load and run the SVCall handler from the bootloader
    mov r0, #(0x2c)
    ldr r0, [r0]
    bx  r0

    ; function should never return
    .endasmfunc

    .end
