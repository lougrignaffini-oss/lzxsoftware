#ifndef __NEWBANKS_H__
#define __NEWBANKS_H__

// New Diver banks (proposed) -- all built as ordinary WavePlusLUT instances,
// same pattern as the existing factory banks in BankLayout.h. No new
// architecture, no new RAM beyond the shared lut[] buffer already used by
// every bank, plus one small static cache (Elementary CA) noted inline.
//
// Controls follow the factory-bank convention: the H/V Phase sliders keep
// their normal job of positioning/scrolling the image, and each bank's two
// shape parameters live on Alt A / Alt B (hold Clear and move the H / V
// slider). altA_default / altB_default are the values loaded when the bank
// is selected.

#include "WavePlusLUT.h"
#include "globals.h"
#include "main.h"

#include <cmath>
#include <cstdint>

struct NewBanks
{
    // ------------------------------------------------------------------
    // 1. Logistic Map Bifurcation
    //    Alt A: pans the center r-value being swept across the ramp
    //    Alt B: zooms the r-window width (narrow = deep zoom into a
    //           bifurcation/chaos band, wide = the full classic diagram)
    //    Outputs a single orbit sample rather than an average -- averaging
    //    the branches of a periodic orbit collapses it to a flat line.
    // ------------------------------------------------------------------
    DiverBankBase* LogisticMap = new WavePlusLUT(
        {.altA_default = 0.6f,
         .altB_default = 0.5f,
         .scrollrange_default = ScrollRange::Hare,
         .deinterlace_mode = 0,
         .update_style = WavePlusLUT::UpdateStyle::PerFrame},
        [](WavePlusLUT::Lookup& l, DiverUIState& state)
        {
            float center = 2.8f + state.param_altA * 1.19f;        // 2.8..3.99
            float width  = 0.2f + state.param_altB * 1.4f;         // zoom window
            float r_min = center - width * 0.5f;
            float r_max = center + width * 0.5f;
            // r > 4.0 diverges to infinity -- clamp so the window can never
            // cross that boundary regardless of Alt A/B combination
            if (r_min < 0.0f) r_min = 0.0f;
            if (r_max > 3.999f) r_max = 3.999f;
            if (r_min > r_max) r_min = r_max;

            float pos = float(l.i) / float(l.vres);
            float r = r_min + pos * (r_max - r_min);

            float x = 0.5f;
            for (int n = 0; n < 61; n++)
            {
                x = r * x * (1.0f - x);
            }
            return uint16_t(x * DAC_MAX_VALUE) & 1023;
        }
    );

    // ------------------------------------------------------------------
    // 2. Weierstrass Function
    //    Alt A: b -- frequency multiplier per octave (smooth -> jagged)
    //    Alt B: a -- amplitude falloff per octave (soft -> harsh detail)
    //    Octaves stop at Nyquist (vres/2 cycles per ramp): anything above
    //    that only aliases, and the large cosf arguments it needs lose all
    //    float precision and are slow to range-reduce on the M4.
    // ------------------------------------------------------------------
    DiverBankBase* Weierstrass = new WavePlusLUT(
        {.altA_default = 0.2f,
         .altB_default = 0.5f,
         .scrollrange_default = ScrollRange::Hare,
         .deinterlace_mode = 0,
         .update_style = WavePlusLUT::UpdateStyle::PerFrame},
        [](WavePlusLUT::Lookup& l, DiverUIState& state)
        {
            float b = 2.0f + state.param_altA * 10.0f;     // 2..12
            float a = 0.2f + state.param_altB * 0.6f;      // 0.2..0.8
            float nyquist = float(l.vres) * 0.5f;

            float ang = (float(l.i) / float(l.vres)) * 2.0f * float(M_PI);
            float s = 0.0f, amp = 1.0f, freq = 1.0f, norm = 0.0f;
            for (int oct = 0; oct < 8 && freq < nyquist; oct++)
            {
                s += amp * cosf(freq * ang);
                norm += amp;
                amp *= a;
                freq *= b;
            }
            float val = (s / norm + 1.0f) * 0.5f;          // -norm..norm -> 0..1
            return uint16_t(val * DAC_MAX_VALUE) & 1023;
        }
    );

    // ------------------------------------------------------------------
    // 3. Takagi (Blancmange) Function
    //    Alt A: octave count (smooth -> maximally jagged)
    //    Alt B: per-octave amplitude decay (classic curve is 0.5 -- sweeping
    //           it changes how quickly fine detail fades)
    // ------------------------------------------------------------------
    DiverBankBase* Takagi = new WavePlusLUT(
        {.altA_default = 0.5f,
         .altB_default = 0.5f,
         .scrollrange_default = ScrollRange::Hare,
         .deinterlace_mode = 0,
         .update_style = WavePlusLUT::UpdateStyle::PerFrame},
        [](WavePlusLUT::Lookup& l, DiverUIState& state)
        {
            int octaves = 1 + int(state.param_altA * 8.0f);     // 1..9
            float decay = 0.2f + state.param_altB * 0.6f;       // 0.2..0.8

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
    //    Alt A: numeral base (2..8) the index is summed in
    //    Alt B: modulus the digit-sum is taken against (2..8) -- base 2,
    //           modulus 2 (both at 0, the default) is the classic
    //           Thue-Morse sequence
    // ------------------------------------------------------------------
    DiverBankBase* ThueMorse = new WavePlusLUT(
        {.altA_default = 0.0f,
         .altB_default = 0.0f,
         .scrollrange_default = ScrollRange::Hare,
         .deinterlace_mode = 0,
         .update_style = WavePlusLUT::UpdateStyle::PerFrame},
        [](WavePlusLUT::Lookup& l, DiverUIState& state)
        {
            int base = 2 + int(state.param_altA * 6.0f);      // 2..8
            int mod  = 2 + int(state.param_altB * 6.0f);      // 2..8

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
    //    Alt A: rule number (even rules 0..254 -- odd rules switch every
    //           empty cell on, which just flashes the whole line)
    //    Alt B: generations forward -- "zoom into time"
    //    Computed once per field into a static cache on l.i==0, then
    //    served from cache for the rest of that field's lut[] fill.
    // ------------------------------------------------------------------
    DiverBankBase* ElementaryCA = new WavePlusLUT(
        {.altA_default = 15.0f / 127.0f, // rule 30
         .altB_default = 0.5f,
         .scrollrange_default = ScrollRange::Hare,
         .deinterlace_mode = 0,
         .update_style = WavePlusLUT::UpdateStyle::PerFrame},
        [](WavePlusLUT::Lookup& l, DiverUIState& state)
        {
            static uint8_t row[MAX_BUFFER_SIZE];
            static uint8_t rowbuf[MAX_BUFFER_SIZE];

            if (l.i == 0)
            {
                uint8_t rule = uint8_t(state.param_altA * 127.0f + 0.5f) << 1;
                int gens = 1 + int(state.param_altB * 150.0f);
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
                        uint8_t left  = row[i == 0 ? n - 1 : i - 1];
                        uint8_t here  = row[i];
                        uint8_t right = row[i == n - 1 ? 0 : i + 1];
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
    // 6. Mandelbrot Slice
    //    Alt A: c_im -- vertical position/offset in the complex plane
    //    Alt B: zoom -- width of the c_re window swept across the ramp
    // ------------------------------------------------------------------
    DiverBankBase* MandelbrotSlice = new WavePlusLUT(
        {.altA_default = 0.5f,
         .altB_default = 0.8f,
         .scrollrange_default = ScrollRange::Hare,
         .deinterlace_mode = 0,
         .update_style = WavePlusLUT::UpdateStyle::PerFrame},
        [](WavePlusLUT::Lookup& l, DiverUIState& state)
        {
            float c_im = -0.8f + state.param_altA * 1.6f;          // -0.8..0.8
            float window = 0.3f + state.param_altB * 3.2f;         // zoom width
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
    // 7. Julia Slice
    //    Alt A: c_re (-1.3..0.3)
    //    Alt B: c_im (-0.7..0.7)
    //    (standard direct Julia-set navigation -- z sweeps the ramp, c is
    //    set live by the two alt params; the c ranges are kept mostly
    //    inside the Mandelbrot set, where the Julia set is connected --
    //    outside it the whole line escapes at once and goes flat)
    // ------------------------------------------------------------------
    DiverBankBase* JuliaSlice = new WavePlusLUT(
        {.altA_default = 0.3f,
         .altB_default = 0.6f,
         .scrollrange_default = ScrollRange::Hare,
         .deinterlace_mode = 0,
         .update_style = WavePlusLUT::UpdateStyle::PerFrame},
        [](WavePlusLUT::Lookup& l, DiverUIState& state)
        {
            float c_re = -1.3f + state.param_altA * 1.6f;    // -1.3..0.3
            float c_im = -0.7f + state.param_altB * 1.4f;    // -0.7..0.7

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
