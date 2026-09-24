#ifndef __NEWBANKS_H__
#define __NEWBANKS_H__

// New Diver banks (proposed) -- all built as ordinary WavePlusLUT instances,
// same pattern as the existing factory banks in BankLayout.h. No new
// architecture, no new RAM beyond the shared lut[] buffer already used by
// every bank, and two small static caches (CA, Feedback Comb) noted inline.
//
// Per the real hardware panel, only H Phase and V Phase are physical
// continuous controls -- Alt A / Alt B are not exposed as panel knobs, so
// every bank below reads state.param_hphase / state.param_vphase instead.

#include "WavePlusLUT.h"
#include "globals.h"
#include "main.h"

#include <cmath>
#include <cstdint>

struct NewBanks
{
    // ------------------------------------------------------------------
    // 1. Logistic Map Bifurcation
    //    H Phase: pans the center r-value being swept across the ramp
    //    V Phase: zooms the r-window width (narrow = deep zoom into a
    //             bifurcation/chaos band, wide = the full classic diagram)
    // ------------------------------------------------------------------
    DiverBankBase* LogisticMap = new WavePlusLUT(
        {.altA_default = 0.0f,
         .altB_default = 0.0f,
         .scrollrange_default = ScrollRange::Hare,
         .deinterlace_mode = 0,
         .update_style = WavePlusLUT::UpdateStyle::PerFrame},
        [](WavePlusLUT::Lookup& l, DiverUIState& state)
        {
            float center = 2.4f + state.param_hphase * 1.59f;         // ~2.4..3.99
            float width  = 0.05f + state.param_vphase * 1.54f;        // zoom window
            float r_min = center - width * 0.5f;
            float r_max = center + width * 0.5f;
            // r > 4.0 diverges to infinity -- clamp so the window can never
            // cross that boundary regardless of H/V Phase combination
            if (r_min < 0.0f) r_min = 0.0f;
            if (r_max > 3.999f) r_max = 3.999f;
            if (r_min > r_max) r_min = r_max;

            float pos = float(l.i) / float(l.vres);
            float r = r_min + pos * (r_max - r_min);

            float x = 0.5f;
            for (int n = 0; n < 60; n++)
            {
                x = r * x * (1.0f - x);
            }
            float acc = 0.0f;
            for (int n = 0; n < 8; n++)
            {
                x = r * x * (1.0f - x);
                acc += x;
            }
            return uint16_t((acc / 8.0f) * DAC_MAX_VALUE) & 1023;
        }
    );

    // ------------------------------------------------------------------
    // 2. Weierstrass Function
    //    H Phase: b -- frequency multiplier per octave (smooth -> jagged)
    //    V Phase: a -- amplitude falloff per octave (soft -> harsh detail)
    // ------------------------------------------------------------------
    DiverBankBase* Weierstrass = new WavePlusLUT(
        {.altA_default = 0.0f,
         .altB_default = 0.0f,
         .scrollrange_default = ScrollRange::Hare,
         .deinterlace_mode = 0,
         .update_style = WavePlusLUT::UpdateStyle::PerFrame},
        [](WavePlusLUT::Lookup& l, DiverUIState& state)
        {
            float b = 2.0f + state.param_hphase * 10.0f;   // 2..12
            float a = 0.2f + state.param_vphase * 0.6f;    // 0.2..0.8

            float ang = (float(l.i) / float(l.vres)) * 2.0f * float(M_PI);
            float s = 0.0f, amp = 1.0f, freq = 1.0f;
            for (int oct = 0; oct < 8; oct++)
            {
                s += amp * cosf(freq * ang);
                amp *= a;
                freq *= b;
            }
            float val = (s + 2.0f) / 4.0f;
            if (val < 0.0f) val = 0.0f;
            if (val > 1.0f) val = 1.0f;
            return uint16_t(val * DAC_MAX_VALUE) & 1023;
        }
    );

    // ------------------------------------------------------------------
    // 3. Takagi (Blancmange) Function
    //    H Phase: octave count (smooth -> maximally jagged)
    //    V Phase: per-octave amplitude decay (normally 0.5 -- sweeping it
    //             changes how quickly fine detail fades)
    // ------------------------------------------------------------------
    DiverBankBase* Takagi = new WavePlusLUT(
        {.altA_default = 0.0f,
         .altB_default = 0.0f,
         .scrollrange_default = ScrollRange::Hare,
         .deinterlace_mode = 0,
         .update_style = WavePlusLUT::UpdateStyle::PerFrame},
        [](WavePlusLUT::Lookup& l, DiverUIState& state)
        {
            int octaves = 1 + int(state.param_hphase * 8.0f);   // 1..9
            float decay = 0.2f + state.param_vphase * 0.6f;     // 0.2..0.8

            float pos = float(l.i) / float(l.vres);
            float s = 0.0f, amp = 1.0f, freq = 1.0f, norm = 0.0f;
            for (int oct = 0; oct < octaves; oct++)
            {
                float x = pos * freq;
                float tri = fabsf(x - roundf(x)) * 2.0f;
                s += amp * tri;
                norm += amp;
                amp *= decay;
                freq *= 2.0f;
            }
            float val = (norm > 0.0f) ? (s / norm) : 0.0f;
            return uint16_t(val * DAC_MAX_VALUE) & 1023;
        }
    );

    // ------------------------------------------------------------------
    // 4. Thue-Morse Bit-Parity (generalized to variable base/modulus)
    //    H Phase: numeral base (2..8) the index is summed in
    //    V Phase: modulus the digit-sum is taken against (2..8) -- at
    //             base==modulus this is the classic Thue-Morse sequence
    // ------------------------------------------------------------------
    DiverBankBase* ThueMorse = new WavePlusLUT(
        {.altA_default = 0.0f,
         .altB_default = 0.0f,
         .scrollrange_default = ScrollRange::Hare,
         .deinterlace_mode = 0,
         .update_style = WavePlusLUT::UpdateStyle::PerFrame},
        [](WavePlusLUT::Lookup& l, DiverUIState& state)
        {
            int base = 2 + int(state.param_hphase * 6.0f);      // 2..8
            int mod  = 2 + int(state.param_vphase * 6.0f);      // 2..8

            int n = l.i;
            int digitsum = 0;
            while (n > 0)
            {
                digitsum += n % base;
                n /= base;
            }
            bool on = (digitsum % mod) >= (mod / 2);
            return uint16_t(on ? DAC_MAX_VALUE : 0) & 1023;
        }
    );

    // ------------------------------------------------------------------
    // 5. Elementary Cellular Automaton
    //    H Phase: rule number (0..255)
    //    V Phase: generations forward -- "zoom into time"
    //    Computed once per field into a static cache on l.i==0, then
    //    served from cache for the rest of that field's lut[] fill.
    // ------------------------------------------------------------------
    DiverBankBase* ElementaryCA = new WavePlusLUT(
        {.altA_default = 0.0f,
         .altB_default = 0.0f,
         .scrollrange_default = ScrollRange::Hare,
         .deinterlace_mode = 0,
         .update_style = WavePlusLUT::UpdateStyle::PerFrame},
        [](WavePlusLUT::Lookup& l, DiverUIState& state)
        {
            static uint8_t row[MAX_BUFFER_SIZE];
            static uint8_t rowbuf[MAX_BUFFER_SIZE];

            if (l.i == 0)
            {
                uint8_t rule = uint8_t(state.param_hphase * 255.0f);
                int gens = 1 + int(state.param_vphase * 150.0f);
                bool rule_bits[8];
                for (int b = 0; b < 8; b++)
                {
                    rule_bits[b] = (rule >> b) & 1;
                }

                uint16_t n = l.vres;
                for (uint16_t i = 0; i < n; i++)
                {
                    row[i] = 0;
                }
                row[n / 2] = 1;

                for (int g = 0; g < gens; g++)
                {
                    for (uint16_t i = 0; i < n; i++)
                    {
                        uint8_t left  = row[(i + n - 1) % n];
                        uint8_t here  = row[i];
                        uint8_t right = row[(i + 1) % n];
                        uint8_t idx = (left << 2) | (here << 1) | right;
                        rowbuf[i] = rule_bits[idx] ? 1 : 0;
                    }
                    for (uint16_t i = 0; i < n; i++)
                    {
                        row[i] = rowbuf[i];
                    }
                }
            }

            return uint16_t(row[l.i] ? DAC_MAX_VALUE : 0) & 1023;
        }
    );

    // ------------------------------------------------------------------
    // (Feedback Comb removed -- replaced by the Heart/Star shape banks,
    // see ShapeBank.h. It settled to a static image too quickly to be
    // interesting as a live-performance bank; see conversation notes.)
    // ------------------------------------------------------------------

    // ------------------------------------------------------------------
    // 7. Mandelbrot Slice
    //    H Phase: c_im -- vertical position/offset in the complex plane
    //    V Phase: zoom -- width of the c_re window swept across the ramp
    // ------------------------------------------------------------------
    DiverBankBase* MandelbrotSlice = new WavePlusLUT(
        {.altA_default = 0.0f,
         .altB_default = 0.0f,
         .scrollrange_default = ScrollRange::Hare,
         .deinterlace_mode = 0,
         .update_style = WavePlusLUT::UpdateStyle::PerFrame},
        [](WavePlusLUT::Lookup& l, DiverUIState& state)
        {
            float c_im = -1.2f + state.param_hphase * 2.4f;        // -1.2..1.2
            float window = 0.05f + state.param_vphase * 3.45f;     // zoom width
            float c_re_center = -0.75f;

            float pos = float(l.i) / float(l.vres);
            float c_re = (c_re_center - window * 0.5f) + pos * window;

            const int max_iter = 60;
            float zr = 0.0f, zi = 0.0f;
            int iter = 0;
            while (iter < max_iter && (zr * zr + zi * zi) < 4.0f)
            {
                float zr_new = zr * zr - zi * zi + c_re;
                float zi_new = 2.0f * zr * zi + c_im;
                zr = zr_new;
                zi = zi_new;
                iter++;
            }
            float val = float(iter) / float(max_iter);
            return uint16_t(val * DAC_MAX_VALUE) & 1023;
        }
    );

    // ------------------------------------------------------------------
    // 8. Julia Slice
    //    H Phase: c_re
    //    V Phase: c_im
    //    (standard direct Julia-set navigation -- z sweeps the ramp, c is
    //    fixed live by the two knobs)
    // ------------------------------------------------------------------
    DiverBankBase* JuliaSlice = new WavePlusLUT(
        {.altA_default = 0.0f,
         .altB_default = 0.0f,
         .scrollrange_default = ScrollRange::Hare,
         .deinterlace_mode = 0,
         .update_style = WavePlusLUT::UpdateStyle::PerFrame},
        [](WavePlusLUT::Lookup& l, DiverUIState& state)
        {
            float c_re = -1.0f + state.param_hphase * 2.0f;   // -1.0..1.0
            float c_im = -1.0f + state.param_vphase * 2.0f;   // -1.0..1.0

            float pos = float(l.i) / float(l.vres);
            float zr = pos * 3.0f - 1.5f;
            float zi = 0.0f;

            const int max_iter = 60;
            int iter = 0;
            while (iter < max_iter && (zr * zr + zi * zi) < 4.0f)
            {
                float zr_new = zr * zr - zi * zi + c_re;
                float zi_new = 2.0f * zr * zi + c_im;
                zr = zr_new;
                zi = zi_new;
                iter++;
            }
            float val = float(iter) / float(max_iter);
            return uint16_t(val * DAC_MAX_VALUE) & 1023;
        }
    );
};

NewBanks newBanks;

#endif
