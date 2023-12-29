#include "qam_sync_buffers.h"
#include <cstring>

QAM_Synchroniser_Buffer::QAM_Synchroniser_Buffer(const int _block_size, const int M, const int L) 
:   src_block_size(_block_size*M), 
    ds_block_size(_block_size), 
    us_block_size(_block_size*L),
    ds_factor(M),
    us_factor(L)
{
    constexpr size_t SIMD_ALIGN = 32;
    data_allocate = AllocateJoint(
        x_raw,                  BufferParameters(src_block_size, SIMD_ALIGN),
        x_in,                   BufferParameters(src_block_size, SIMD_ALIGN),
        // Downsampled PLL
        x_downsampled,          BufferParameters(ds_block_size, SIMD_ALIGN),
        x_ac,                   BufferParameters(ds_block_size, SIMD_ALIGN),
        x_agc,                  BufferParameters(ds_block_size, SIMD_ALIGN),
        x_pll_out,              BufferParameters(ds_block_size, SIMD_ALIGN),
        error_pll,              BufferParameters(ds_block_size, SIMD_ALIGN),
        // Upsampled TED
        x_upsampled,            BufferParameters(us_block_size, SIMD_ALIGN),
        y_sym_out,              BufferParameters(us_block_size, SIMD_ALIGN),
        trig_zero_crossing,     BufferParameters(us_block_size, SIMD_ALIGN),
        trig_ted_clock,         BufferParameters(us_block_size, SIMD_ALIGN),
        error_ted,              BufferParameters(us_block_size, SIMD_ALIGN),
        trig_integrator_dump,   BufferParameters(us_block_size, SIMD_ALIGN),
        // Output
        y_out,                  BufferParameters(us_block_size, SIMD_ALIGN)
    );
}

bool QAM_Synchroniser_Buffer::CopyFrom(QAM_Synchroniser_Buffer& in) {
    if (in.Size() != Size()) {
        return false;
    } 

    const size_t N = Size();
    std::memcpy(data_allocate.data(), in.data_allocate.data(), N);
    return true;
}