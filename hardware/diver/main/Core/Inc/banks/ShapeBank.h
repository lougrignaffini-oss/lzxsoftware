#ifndef __SHAPEBANK_H__
#define __SHAPEBANK_H__

// Static shape banks (Heart, Star) -- the one pair of banks in this set that
// needs "true 2D" output instead of the additive H(x)+V(y) model every
// WavePlusLUT bank uses. A heart/star silhouette isn't separable as
// f(x)+g(y), so every line needs its own H content.
//
// How it works with the stock video path
// --------------------------------------
// data_transmitted_handler (main.cpp) runs at the end of each line's DMA
// transfer and arms the next line from hwave[waveReadPtr], starting at
// offset hphase_cv[waveReadPtr][linecnt] % hres, with vwave[waveReadPtr]
// [linecnt] as the V-DAC word. The hsync ISR (EXTI, priority 0) runs at the
// start of each line, before that handler, so during line N this bank:
//   1. picks whichever of its two buffers (kBufA / kBufB) is NOT the one the
//      DMA is streaming right now,
//   2. updates it to line N+1's row, rewriting only the samples whose
//      inside/outside state changed since that buffer was last drawn
//      (two lines ago -- usually just a few cells at each edge),
//   3. writes that buffer's hphase_cv / vwave entries for index linecnt,
//   4. points waveReadPtr at it, so the handler arms it for line N+1.
// The buffer being streamed is never written.
//
// Timing: the ISR work is integer-only, and a line is ~63.5us (~6.9k cycles
// @108MHz). In a host simulation of the heart/star at several sizes, mirror
// and phase settings, a line rewrites 13 samples on average and 196 at
// most; the -Og fill loop is 4 instructions per sample (~6 cycles), so
// ~1.2k cycles worst case. A full redraw (524 samples, ~3k cycles) happens
// only on the first two lines after the bank is selected and when Invert
// changes. All geometry (scaling the 64x64 bitmap into spans) is done in
// OnActivate / OnOddField, in the main loop, into a double-buffered Layout
// that the ISR swaps to atomically.
//
// Interlace: linecnt runs across the whole frame, first field then second.
// Line L shows picture row 2L (first half of the frame) or
// 2(L - vres/2) + 1 (second half) -- the same mapping WavePlusLUT's
// interlace_mode applies to its vertical data.
//
// Controls (same conventions as the factory banks):
//   H / V Phase, Scroll X / Y  - position the shape (V Phase CV and H Phase
//                                CV work too)
//   Alt A (Clear + H)          - size (20%..100% of picture height)
//   Alt B (Clear + V)          - width: 0.5x..2x, 0.5 = square on a 4:3
//                                display
//   Invert                     - swaps shape and background levels
//   Mirror X / Mirror Y        - two half-size mirrored copies, like the
//                                ramp banks
// V output: a vertical ramp that follows V Phase (like the linear ramp
// bank), so V stays useful as a second signal.
//
// Still unverified on real hardware -- check with a scope/logic analyzer
// that the hsync ISR finishes well before the next hsync, especially on
// lines 0-2 where DiverUI::OnInterruptHSync also polls the ADC.

#include "DiverBankBase.h"
#include "DiverUIState.h"
#include "globals.h"
#include "main.h"

#include <cmath>
#include <cstdint>
#include <cstring>

#define SHAPE_GRID_N 64
#define SHAPE_MAX_SPANS 4 // bitmaps have at most 2 runs per row; mirror X doubles that

struct ShapeBank : public DiverBankBase
{
    // The two buffer slots this bank alternates between, one line at a time.
    // While a ShapeBank is selected nothing else renders into hwave/vwave/
    // hphase_cv, so any two of the NUM_BUFFERS slots will do.
    static const uint8_t kBufA = 2;
    static const uint8_t kBufB = 3;

    // Filled spans of one grid row, flattened as sorted, disjoint boundary
    // pairs: [x0, x1, x0, x1, ...] in samples.
    struct Spans
    {
        uint8_t n;
        uint16_t b[2 * SHAPE_MAX_SPANS];
    };

    struct Layout
    {
        uint16_t top;    // first sample row covered by the shape
        uint16_t height; // sample rows covered (0 = nothing to draw)
        Spans rows[SHAPE_GRID_N];
    };

    // What one of the two line buffers currently holds.
    struct BufState
    {
        bool valid; // false = unknown contents, needs a full render
        bool invert;
        Spans spans;
    };

    // Only one bank is selected at a time, so all ShapeBanks share the
    // layout double buffer and the line-buffer state.
    static inline Layout layouts[2];
    static inline volatile uint8_t layoutIdx = 0;
    static inline BufState bufState[2];
    static inline const Spans kNoSpans = {0, {0}};

    const uint8_t (*bitmap)[SHAPE_GRID_N];

    // Inputs the current layout was built from
    float builtAltA;
    float builtAltB;
    uint8_t builtMirrorX;
    uint16_t builtHres;
    uint16_t builtVres;

    explicit ShapeBank(const uint8_t (*bitmap_)[SHAPE_GRID_N])
        : bitmap(bitmap_)
    {
    }

    void Init() override
    {
        // hres/vres aren't known yet at Init time; the layout is built in
        // OnActivate. Until then layouts[] is zeroed (height 0 = draw nothing).
    }

    void OnActivate(DiverUIState& state) override
    {
        state.param_altA = 0.75f;
        state.param_altB = 0.5f;
        state.scrollx_range = ScrollRange::Tortoise;
        state.scrolly_range = ScrollRange::Tortoise;

        BuildLayout(state);
        // The buffers hold whatever the previous bank left there.
        bufState[0].valid = false;
        bufState[1].valid = false;
    }

    void OnOddField(DiverUIState& state) override
    {
        if (state.param_altA != builtAltA || state.param_altB != builtAltB || state.mirrorx != builtMirrorX ||
            hres != builtHres || vres != builtVres)
        {
            BuildLayout(state);
        }
    }

    void OnInterruptFrameStart(DiverUIState& state) override
    {
        // Keep the H Phase CV capture moving, as WavePlusLUT does. There is
        // no whole-frame buffer swap: this bank swaps buffers every line.
        if (state.captureEnable)
        {
            sampleReadPtr = sampleWritePtr;
            sampleWritePtr = (sampleWritePtr + 1) % NUM_BUFFERS;
        }
    }

    void OnInterruptHSync(DiverUIState& state) override
    {
        const uint32_t h = hres;
        const uint32_t v = vres;

        // Never touch the buffer the DMA is streaming for the current line.
        uint8_t target = (waveReadPtr == kBufA) ? kBufB : kBufA;
        BufState& buf = bufState[(target == kBufA) ? 0 : 1];
        const Layout& layout = layouts[layoutIdx];

        // Prepare the line after this one.
        uint32_t next_line = uint32_t(linecnt) + 1;
        uint32_t sample_row = 0;
        const Spans* want = &kNoSpans;

        if (next_line < v)
        {
            // Frame line -> picture row (interlaced), then V phase / mirror,
            // matching WavePlusLUT's vertical sample_index calculation.
            uint32_t half = v >> 1;
            uint32_t y = (next_line < half) ? (next_line << 1) : (((next_line - half) << 1) + 1);

            uint32_t vp = state.scrolly ? state.vphasecnt : state.vphase_slider;
            sample_row = (y + (v - vp) + (v - state.vphase_cv) + VPHASE_OFFSET) % v;

            if (state.mirrory)
            {
                if (sample_row >= half)
                {
                    sample_row = (half - (sample_row + 1 - half)) << 1;
                }
                else
                {
                    sample_row = sample_row << 1;
                }
            }

            if (layout.height && sample_row >= layout.top && sample_row < uint32_t(layout.top) + layout.height)
            {
                want = &layout.rows[(sample_row - layout.top) * SHAPE_GRID_N / layout.height];
            }
        }

        uint16_t* dest = hwave[target];
        uint16_t bg = state.invert ? DAC_MAX_VALUE : 0;
        uint16_t fg = state.invert ? 0 : DAC_MAX_VALUE;
        if (!buf.valid || buf.invert != bool(state.invert))
        {
            RenderFull(dest, h, *want, fg, bg);
            buf.valid = true;
            buf.invert = state.invert;
        }
        else
        {
            RenderDiff(dest, h, buf.spans, *want, fg, bg);
        }
        buf.spans = *want;

        // H position: bake H Phase (slider or scroll) and H Phase CV into the
        // DMA start offset, which is equivalent to WavePlusLUT shifting its
        // row by the same amounts.
        uint32_t hp = state.scrollx ? state.hphasecnt : state.hphase_slider;
        uint32_t hcv = samples_hphase_cv[sampleReadPtr][sample_row];
        hphase_cv[target][linecnt] = uint16_t((hcv + (h - hp) + HPHASE_OFFSET) % h);

        vwave[target][linecnt] = uint16_t((sample_row * DAC_MAX_VALUE) / v) & 1023;

        waveReadPtr = target;
    }

  protected:
    // Write samples [a, b) of the line, in both copies (the DMA can start
    // anywhere in [0, hres) and reads a full line past it -- same layout
    // WavePlusLUT writes), plus the two samples the transfer reaches past
    // 2 * hres.
    static void Fill(uint16_t* dest, uint32_t h, uint32_t a, uint32_t b, uint16_t value)
    {
        uint16_t* p = dest + a;
        uint16_t* q = dest + h + a;
        uint16_t* end = dest + b;
        while (p < end)
        {
            *p++ = value;
            *q++ = value;
        }
        for (uint32_t i = a; i < b && i < 2; i++)
        {
            dest[2 * h + i] = value;
        }
    }

    static void RenderFull(uint16_t* dest, uint32_t h, const Spans& spans, uint16_t fg, uint16_t bg)
    {
        uint32_t pos = 0;
        for (uint8_t i = 0; i < spans.n; i += 2)
        {
            Fill(dest, h, pos, spans.b[i], bg);
            Fill(dest, h, spans.b[i], spans.b[i + 1], fg);
            pos = spans.b[i + 1];
        }
        Fill(dest, h, pos, h, bg);
    }

    // Rewrite only the samples whose inside/outside state differs between
    // the spans the buffer holds and the spans wanted -- usually just a few
    // cells at each edge, since neighbouring grid rows are similar.
    static void RenderDiff(uint16_t* dest, uint32_t h, const Spans& have, const Spans& want, uint16_t fg, uint16_t bg)
    {
        uint8_t i = 0;
        uint8_t j = 0;
        bool in_have = false;
        bool in_want = false;
        uint32_t pos = 0;
        while (i < have.n || j < want.n)
        {
            uint32_t next_have = (i < have.n) ? have.b[i] : 0xFFFFFFFF;
            uint32_t next_want = (j < want.n) ? want.b[j] : 0xFFFFFFFF;
            uint32_t next = (next_have < next_want) ? next_have : next_want;
            if (in_have != in_want && next > pos)
            {
                Fill(dest, h, pos, next, in_want ? fg : bg);
            }
            pos = next;
            if (next_have == next)
            {
                in_have = !in_have;
                i++;
            }
            if (next_want == next)
            {
                in_want = !in_want;
                j++;
            }
        }
    }

    void BuildLayout(DiverUIState& state)
    {
        builtAltA = state.param_altA;
        builtAltB = state.param_altB;
        builtMirrorX = state.mirrorx;
        builtHres = hres;
        builtVres = vres;

        Layout& layout = layouts[layoutIdx ^ 1];
        layout.top = 0;
        layout.height = 0;
        for (int r = 0; r < SHAPE_GRID_N; r++)
        {
            layout.rows[r].n = 0;
        }

        if (hres == 0 || vres == 0)
        {
            layoutIdx ^= 1;
            return;
        }

        // Size: 20%..100% of the picture height.
        int h = int((0.2f + 0.8f * state.param_altA) * float(vres));
        if (h < SHAPE_GRID_N / 2)
        {
            h = SHAPE_GRID_N / 2;
        }
        if (h > int(vres))
        {
            h = vres;
        }

        // Width: square on a 4:3 picture at Alt B = 0.5, 0.5x..2x either side.
        float aspect = exp2f(2.0f * state.param_altB - 1.0f);
        int w = int(float(h) * (float(hres) / float(vres)) * 0.75f * aspect);
        if (w < SHAPE_GRID_N / 2)
        {
            w = SHAPE_GRID_N / 2;
        }

        int left = (int(hres) - w) / 2;
        layout.top = uint16_t((vres - h) / 2);
        layout.height = uint16_t(h);

        for (int r = 0; r < SHAPE_GRID_N; r++)
        {
            int c = 0;
            while (c < SHAPE_GRID_N)
            {
                if (bitmap[r][c] < 128)
                {
                    c++;
                    continue;
                }
                int c0 = c;
                while (c < SHAPE_GRID_N && bitmap[r][c] >= 128)
                {
                    c++;
                }

                int a = left + c0 * w / SHAPE_GRID_N;
                int b = left + c * w / SHAPE_GRID_N;
                if (state.mirrorx)
                {
                    // WavePlusLUT's mirror: output x < hres/2 shows source 2x,
                    // output x >= hres/2 shows source 2(hres - 1 - x).
                    AddSpan(layout.rows[r], a / 2, (b + 1) / 2);
                    AddSpan(layout.rows[r], int(hres) - (b + 1) / 2, int(hres) - a / 2);
                }
                else
                {
                    AddSpan(layout.rows[r], a, b);
                }
            }
        }

        layoutIdx ^= 1; // single byte write: the ISR sees the old or new layout, never a mix
    }

    // Insert [a, b) keeping the row's spans sorted and disjoint (mirrored
    // halves arrive out of order and can touch in the middle).
    static void AddSpan(Spans& spans, int a, int b)
    {
        if (a < 0)
        {
            a = 0;
        }
        if (b > int(hres))
        {
            b = hres;
        }
        if (a >= b)
        {
            return;
        }

        uint16_t x0[SHAPE_MAX_SPANS + 1];
        uint16_t x1[SHAPE_MAX_SPANS + 1];
        uint8_t n = 0;
        bool placed = false;
        for (uint8_t i = 0; i < spans.n; i += 2)
        {
            if (!placed && a < spans.b[i])
            {
                x0[n] = a;
                x1[n] = b;
                n++;
                placed = true;
            }
            x0[n] = spans.b[i];
            x1[n] = spans.b[i + 1];
            n++;
        }
        if (!placed)
        {
            x0[n] = a;
            x1[n] = b;
            n++;
        }

        uint8_t out = 0;
        for (uint8_t i = 0; i < n; i++)
        {
            if (out > 0 && x0[i] <= spans.b[out - 1])
            {
                if (x1[i] > spans.b[out - 1])
                {
                    spans.b[out - 1] = x1[i];
                }
            }
            else if (out < 2 * SHAPE_MAX_SPANS)
            {
                spans.b[out++] = x0[i];
                spans.b[out++] = x1[i];
            }
        }
        spans.n = out;
    }
};

#include "shape_bitmaps.h"

static ShapeBank heartShapeBank(HEART_BITMAP);
static ShapeBank starShapeBank(STAR_BITMAP);

#endif
