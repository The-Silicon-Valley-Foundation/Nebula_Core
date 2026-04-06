#ifndef __NEBULA_SYSTEM_H
#define __NEBULA_SYSTEM_H

static inline void irq_setmask(unsigned int mask) { 
    __asm__ __volatile__ ("csrw mie, %0" : : "r"(mask)); 
}

static inline unsigned int irq_getmask(void) { 
    unsigned int mask; 
    __asm__ __volatile__ ("csrr %0, mie" : "=r"(mask)); 
    return mask; 
}

static inline unsigned int irq_pending(void) { 
    unsigned int pending; 
    __asm__ __volatile__ ("csrr %0, mip" : "=r"(pending)); 
    return pending; 
}

static inline void irq_setie(unsigned int ie) { 
    if (ie) __asm__ __volatile__ ("csrsi mstatus, 8"); 
    else __asm__ __volatile__ ("csrci mstatus, 8"); 
}

static inline unsigned int irq_getie(void) { 
    unsigned int ie; 
    __asm__ __volatile__ ("csrr %0, mstatus" : "=r"(ie)); 
    return (ie >> 3) & 1; 
}

static inline void flush_cpu_icache(void) { 
    __asm__ __volatile__ ("fence.i" : : : "memory"); 
}

static inline void flush_cpu_dcache(void) { 
    /* D-Cache vazia pois o Nebula Core já a gere no RTL */
}

#endif /* __NEBULA_SYSTEM_H */