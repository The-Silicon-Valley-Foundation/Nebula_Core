.global boot_helper
    .section .text

/* * Função fantasma para satisfazer o linker do LiteX.
 * Em processadores single-core, apenas retornamos imediatamente.
 */
boot_helper:
    ret