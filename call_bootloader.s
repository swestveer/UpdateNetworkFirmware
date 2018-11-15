    ; Author: Alex Wranovsky

    ; function to call the bootloader
    .global call_bootloader
    .thumbfunc call_bootloader
call_bootloader: .asmfunc
    mov r0, #(0x2c)
    ldr r0, [r0]
    bx  r0
    ; function should never return
    .endasmfunc

    .end
