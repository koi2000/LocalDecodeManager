#define ADDR1 (*(volatile long*)addr)

static inline int sync_test_and_set_bit(int nr, volatile unsigned long* addr) {
    int oldbit;
    asm volatile("lock; btsl %2,%1\n\tsbbl %0,%0" : "=r"(oldbit), "+m"(ADDR1) : "Ir"(nr) : "memory");
    return oldbit;
}