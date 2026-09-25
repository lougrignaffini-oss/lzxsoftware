#ifndef __SHAPEBANK_H__
#define __SHAPEBANK_H__

// *** NOT CURRENTLY WIRED INTO banks[] (BankLayout.h) ***
// Audit against the stock firmware found blocking problems with the
// per-line approach below; it needs reworking before it is re-enabled:
//   - Interlace: linecnt runs across the whole frame (both fields), so
//     linecnt / band_lines draws the top half of the shape in one field and
//     the bottom half in the other, interleaved. WavePlusLUT reorders its
//     vertical data (interlace_mode) for exactly this reason.
//   - ISR budget: a line is ~63.5us (~6.9k cycles at 108MHz) -- the 48.6us
//     below is only the active part. RegenerateRow() x2 on a band change is
//     ~1k float interpolations (~40k cycles, several lines) inside the
//     priority-0 EXTI handler, so hsyncs are missed and linecnt drifts.
//   - Buffers: OnInterruptFrameStart/OnOddField never swap buffers or write
//     hphase_cv, so the DMA start offset (data_transmitted_handler) is left
//     over from the previous bank; and writing hwave[waveReadPtr] from the
//     hsync ISR races the DMA already streaming that buffer (and overwrites
//     the hwave[hres + HBLANK] slot that carries the V-DAC word).
// Likely direction: build the whole frame in OnOddField into
// hwave/vwave/hphase_cv[waveWritePtr] with interlace-aware row mapping and
// swap in OnInterruptFrameStart, like WavePlusLUT.

// Static shape banks (Heart, Star) -- the ONE pair of banks in this set
// that needs the "true 2D" architecture instead of the additive H(x)+V(y)
// model every other bank uses. A heart/star silhouette isn't separable as
// f(x)+g(y), so it can't be built with WavePlusLUT at all.
//
// *** UNVERIFIED ON REAL HARDWARE -- READ BEFORE FLASHING ***
// This writes directly into hwave[waveReadPtr] from OnInterruptHSync (the
// per-line hsync ISR, driven off the external genlock reference via
// linecnt), following the same "poke the active read buffer from an ISR"
// pattern the stock firmware already uses to inject the V-channel value in
// data_transmitted_handler (see main.cpp). Estimated timing, from the real
// TIM1 config (27MHz HSE -> 108MHz core, tim1period=10 -> ~92.6ns/sample,
// hres=525 -> ~48.6us/line):
//   - every line: one lerp over hres samples (~31.5us at 100MHz, fixed-point
//     recommended over float for margin)
//   - every ~8 lines (vres/GRID_N band): one row re-interpolation from the
//     64-wide bitmap up to hres samples
// That leaves real but not huge margin against the ~48.6us/line budget,
// and depends on the relative timing between the EXTI15_10 (hsync/genlock,
// priority 0) and DMA2_Stream5 (transfer-complete, priority 1) interrupts,
// which was reasoned from the source but NOT measured on a scope. Verify
// with a logic analyzer on real hardware before trusting this in a set.
//
// No per-field simulation cost (unlike Gray-Scott) -- these are fixed
// bitmaps, so OnOddField does nothing. That removes one whole category of
// risk Gray-Scott had, which is why this is a safer target for the
// technique than reaction-diffusion was.
//
// V channel is held at a constant mid-scale value -- these banks put the
// entire image on H, so V does not add a second independent signal the way
// it does on every other bank in this set. Worth knowing if you patch this
// through DSG3 expecting a real second channel.

#include "DiverBankBase.h"
#include "DiverUIState.h"
#include "globals.h"
#include "main.h"

#include <cstdint>

#define SHAPE_GRID_N 64

struct ShapeBank : public DiverBankBase
{
    const uint8_t (*bitmap)[SHAPE_GRID_N];

    uint16_t row_curr[MAX_BUFFER_SIZE];
    uint16_t row_next[MAX_BUFFER_SIZE];
    int cached_band;
    int band_lines;

    explicit ShapeBank(const uint8_t (*bitmap_)[SHAPE_GRID_N])
        : bitmap(bitmap_), cached_band(-1), band_lines(1)
    {
    }

    void Init() override
    {
    }

    void OnActivate(DiverUIState& state) override
    {
        band_lines = vres / SHAPE_GRID_N;
        if (band_lines < 1)
        {
            band_lines = 1;
        }
        cached_band = -1; // force a refresh on the next OnInterruptHSync
    }

    static uint16_t InterpSample(const uint8_t* row, uint16_t out_i, uint16_t out_len)
    {
        float pos = (float(out_i) / float(out_len)) * float(SHAPE_GRID_N - 1);
        int i0 = int(pos);
        int i1 = i0 + 1;
        if (i1 > SHAPE_GRID_N - 1)
        {
            i1 = SHAPE_GRID_N - 1;
        }
        float frac = pos - float(i0);
        float v = float(row[i0]) * (1.0f - frac) + float(row[i1]) * frac;
        return uint16_t((v / 255.0f) * DAC_MAX_VALUE) & 1023;
    }

    void RegenerateRow(uint16_t* dest, int grid_row) const
    {
        if (grid_row < 0)
        {
            grid_row = 0;
        }
        if (grid_row > SHAPE_GRID_N - 1)
        {
            grid_row = SHAPE_GRID_N - 1;
        }
        const uint8_t* row = bitmap[grid_row];
        for (uint16_t i = 0; i < hres; i++)
        {
            dest[i] = InterpSample(row, i, hres);
        }
    }

    void OnInterruptHSync(DiverUIState& state) override
    {
        int band = linecnt / band_lines;
        if (band != cached_band)
        {
            cached_band = band;
            RegenerateRow(row_curr, band);
            RegenerateRow(row_next, band + 1);
        }

        float frac = float(linecnt % band_lines) / float(band_lines);
        for (uint16_t i = 0; i < hres; i++)
        {
            uint16_t blended = uint16_t(float(row_curr[i]) * (1.0f - frac) + float(row_next[i]) * frac);
            hwave[waveReadPtr][i] = blended;
            hwave[waveReadPtr][hres + i] = blended;
        }

        // V held neutral (mid-scale) -- see note at top of file.
        vwave[waveReadPtr][linecnt] = DAC_MAX_VALUE / 2;
    }

    void OnInterruptFrameStart(DiverUIState& state) override
    {
        // No double-buffer swap: content is regenerated per-line directly
        // into the active read buffer, so there's no "next field" to swap
        // in the way WavePlusLUT's OnOddField/OnInterruptFrameStart pair
        // does. waveReadPtr/waveWritePtr are simply left as whatever the
        // last WavePlusLUT-style bank set them to.
    }

    void OnOddField(DiverUIState& state) override
    {
        // Static shape -- nothing to precompute per field.
    }
};

#include "shape_bitmaps.h"

static ShapeBank heartShapeBank(HEART_BITMAP);
static ShapeBank starShapeBank(STAR_BITMAP);

#endif
