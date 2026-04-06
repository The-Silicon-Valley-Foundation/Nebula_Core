.global _start
    .section .text

_start:
    /* 1. Inicializa o Stack Pointer (Pilha) no topo da RAM alocada */
    la sp, _fstack

    /* 2. Salta para a função main() da BIOS do LiteX */
    call main

    /* 3. Se a main() retornar por algum motivo, prende o CPU num loop infinito */
1:
    j 1b