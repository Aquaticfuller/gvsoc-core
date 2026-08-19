/*
 * Fast floating-point environment access for the flexfloat FP model.
 *
 * WHY. Emulating one guest FP operation costs a stack of libc <fenv.h> calls:
 * the ISS sets the RISC-V rounding mode, flexfloat reads it, saves/restores the
 * accrued exception flags around its own rounding arithmetic, and ff_fma
 * additionally flips the hardware mode to UPWARD/DOWNWARD and back so that the
 * double-precision backend rounds correctly into a narrower destination format.
 * On a 256-core GEMM these calls were ~30% of simulation time.
 *
 * The cost is not the bookkeeping, it is that glibc's fesetround() writes BOTH
 * the x87 control word and MXCSR (fnstcw + fldcw + stmxcsr + ldmxcsr), and
 * feraiseexcept() actually executes a trapping FP operation. `fldcw` and
 * `ldmxcsr` serialise the FP pipeline, so each one costs far more than the
 * arithmetic it guards.
 *
 * WHAT. On x86-64 every double the model computes with lives in an XMM
 * register, so only MXCSR is architecturally relevant -- the x87 control word
 * is dead weight. These helpers therefore touch MXCSR alone, and skip the
 * write entirely when the register already holds the wanted value. Same
 * observable state, a third of the instructions.
 *
 * The flag and rounding encodings are shared between <fenv.h> and MXCSR on
 * x86: the exception bits (FE_INVALID..FE_INEXACT) sit in MXCSR[5:0] in the
 * same order, and the rounding field is the FE_* value shifted left by 3
 * (MXCSR[14:13]) -- which is exactly how glibc itself derives it.
 *
 * Set GVSOC_FP_FAST_FENV=0 to fall back to the libc calls (used to prove the
 * two paths produce identical results).
 */

#ifndef FF_FENV_FAST_H
#define FF_FENV_FAST_H

#include <fenv.h>
#include <stdlib.h>

#if defined(__x86_64__) && !defined(FF_FENV_NO_FAST)
#define FF_FENV_FAST_SUPPORTED 1
#endif

/* MXCSR field layout (Intel SDM Vol.1 10.2.3): [5:0] sticky exception flags,
 * [14:13] rounding control. */
#define FF_MXCSR_EXCEPT_MASK 0x3fu
#define FF_MXCSR_ROUND_MASK 0x6000u

static inline int ff_fenv_fast_enabled(void)
{
#ifdef FF_FENV_FAST_SUPPORTED
    static int enabled = -1;
    if (__builtin_expect(enabled < 0, 0))
    {
        const char *env = getenv("GVSOC_FP_FAST_FENV");
        enabled = (env != NULL && env[0] == '0') ? 0 : 1;
    }
    return enabled;
#else
    return 0;
#endif
}

#ifdef FF_FENV_FAST_SUPPORTED

static inline unsigned int ff_mxcsr_read(void)
{
    unsigned int mxcsr;
    __asm__ __volatile__("stmxcsr %0" : "=m"(mxcsr));
    return mxcsr;
}

static inline void ff_mxcsr_write(unsigned int mxcsr)
{
    __asm__ __volatile__("ldmxcsr %0" : : "m"(mxcsr) : "memory");
}

#endif /* FF_FENV_FAST_SUPPORTED */

/* Current rounding mode, as an FE_* value. */
static inline int ff_getround(void)
{
#ifdef FF_FENV_FAST_SUPPORTED
    if (ff_fenv_fast_enabled())
    {
        return (int)((ff_mxcsr_read() & FF_MXCSR_ROUND_MASK) >> 3);
    }
#endif
    return fegetround();
}

/* Select a rounding mode. The read is cheap; the write is the serialising
 * part, so it is skipped when the mode is already the wanted one -- the common
 * case for the ISS-level "restore the default" calls. */
static inline void ff_setround(int mode)
{
#ifdef FF_FENV_FAST_SUPPORTED
    if (ff_fenv_fast_enabled())
    {
        unsigned int cur = ff_mxcsr_read();
        unsigned int want = (cur & ~FF_MXCSR_ROUND_MASK) |
                            (((unsigned int)mode << 3) & FF_MXCSR_ROUND_MASK);
        if (want != cur)
        {
            ff_mxcsr_write(want);
        }
        return;
    }
#endif
    fesetround(mode);
}

static inline void ff_getexceptflag(fexcept_t *flagp, int excepts)
{
#ifdef FF_FENV_FAST_SUPPORTED
    if (ff_fenv_fast_enabled())
    {
        *flagp = (fexcept_t)(ff_mxcsr_read() & (unsigned int)excepts &
                             FF_MXCSR_EXCEPT_MASK);
        return;
    }
#endif
    fegetexceptflag(flagp, excepts);
}

static inline void ff_setexceptflag(const fexcept_t *flagp, int excepts)
{
#ifdef FF_FENV_FAST_SUPPORTED
    if (ff_fenv_fast_enabled())
    {
        unsigned int keep = (unsigned int)excepts & FF_MXCSR_EXCEPT_MASK;
        unsigned int cur = ff_mxcsr_read();
        unsigned int want = (cur & ~keep) | ((unsigned int)*flagp & keep);
        if (want != cur)
        {
            ff_mxcsr_write(want);
        }
        return;
    }
#endif
    fesetexceptflag(flagp, excepts);
}

static inline int ff_testexcept(int excepts)
{
#ifdef FF_FENV_FAST_SUPPORTED
    if (ff_fenv_fast_enabled())
    {
        return (int)(ff_mxcsr_read() & (unsigned int)excepts &
                     FF_MXCSR_EXCEPT_MASK);
    }
#endif
    return fetestexcept(excepts);
}

static inline void ff_clearexcept(int excepts)
{
#ifdef FF_FENV_FAST_SUPPORTED
    if (ff_fenv_fast_enabled())
    {
        unsigned int cur = ff_mxcsr_read();
        unsigned int want = cur & ~((unsigned int)excepts & FF_MXCSR_EXCEPT_MASK);
        if (want != cur)
        {
            ff_mxcsr_write(want);
        }
        return;
    }
#endif
    feclearexcept(excepts);
}

/* Raising is a plain sticky-bit set here. glibc instead executes an FP
 * operation chosen to raise the wanted exception, which is far more expensive
 * and, for this model, indistinguishable in effect. */
static inline void ff_raiseexcept(int excepts)
{
#ifdef FF_FENV_FAST_SUPPORTED
    if (ff_fenv_fast_enabled())
    {
        unsigned int set = (unsigned int)excepts & FF_MXCSR_EXCEPT_MASK;
        unsigned int cur = ff_mxcsr_read();
        if ((cur & set) != set)
        {
            ff_mxcsr_write(cur | set);
        }
        return;
    }
#endif
    feraiseexcept(excepts);
}

#endif /* FF_FENV_FAST_H */
