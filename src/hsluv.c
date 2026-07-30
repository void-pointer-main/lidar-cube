/*
 * HSLuv-C: Human-friendly HSL
 * <https://github.com/hsluv/hsluv-c>
 * <https://www.hsluv.org/>
 *
 * Copyright (c) 2015 Alexei Boronine (original idea, JavaScript implementation)
 * Copyright (c) 2015 Roger Tallada (Obj-C implementation)
 * Copyright (c) 2017 Martin Mitáš (C implementation, based on Obj-C implementation)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include "hsluv.h"

#include <float.h>
#include <math.h>


#if (DATA_TYPE == double)
#define SQRT_F	sqrt
#define POW_F	pow
#define FABS_F	fabs
#define FMOD_F	fmod
#define FLOOR_F	floor
#define CEIL_F	ceil
#define ATAN2_F	atan2
#define COS_F	cos
#define SIN_F	sin
#define CBRT_F	cbrt
#define EXP_F   exp
#else
#define SQRT_F	sqrtf
#define POW_F	powf
#define FABS_F	fabsf
#define FMOD_F	fmodf
#define FLOOR_F	floorf
#define CEIL_F	ceilf
#define ATAN2_F	atan2f
#define COS_F	cosf
#define SIN_F	sinf
#define CBRT_F	cbrtf
#define EXP_F   expf
#endif

#define CLAMP(val, min_val, max_val)        \
    ((val) < (min_val) ? (min_val) : ((val) > (max_val) ? (max_val) : (val)))


typedef struct Triplet_tag Triplet;
struct Triplet_tag {
    DATA_TYPE a;
    DATA_TYPE b;
    DATA_TYPE c;
};

/* for RGB */
static const Triplet m[3] = {
    {  3.24096994190452134377, -1.53738317757009345794, -0.49861076029300328366 },
    { -0.96924363628087982613,  1.87596750150772066772,  0.04155505740717561247 },
    {  0.05563007969699360846, -0.20397695888897656435,  1.05697151424287856072 }
};

/* for XYZ */
static const Triplet m_inv[3] = {
    {  0.41239079926595948129,  0.35758433938387796373,  0.18048078840183428751 },
    {  0.21263900587151035754,  0.71516867876775592746,  0.07219231536073371500 },
    {  0.01933081871559185069,  0.11919477979462598791,  0.95053215224966058086 }
};

static const DATA_TYPE ref_u = 0.19783000664283680764;
static const DATA_TYPE ref_v = 0.46831999493879100370;

static const DATA_TYPE kappa = 903.29629629629629629630;
static const DATA_TYPE epsilon = 0.00885645167903563082;


typedef struct Bounds_tag Bounds;
struct Bounds_tag {
    DATA_TYPE a;
    DATA_TYPE b;
};


static void
get_bounds(DATA_TYPE l, Bounds bounds[6])
{
    DATA_TYPE tl = l + 16.0;
    DATA_TYPE sub1 = (tl * tl * tl) / 1560896.0;
    DATA_TYPE sub2 = (sub1 > epsilon ? sub1 : (l / kappa));
    int channel;
    int t;

    for(channel = 0; channel < 3; channel++) {
        DATA_TYPE m1 = m[channel].a;
        DATA_TYPE m2 = m[channel].b;
        DATA_TYPE m3 = m[channel].c;

        for (t = 0; t < 2; t++) {
            DATA_TYPE top1 = (284517.0 * m1 - 94839.0 * m3) * sub2;
            DATA_TYPE top2 = (838422.0 * m3 + 769860.0 * m2 + 731718.0 * m1) * l * sub2 -  769860.0 * t * l;
            DATA_TYPE bottom = (632260.0 * m3 - 126452.0 * m2) * sub2 + 126452.0 * t;

            bounds[channel * 2 + t].a = top1 / bottom;
            bounds[channel * 2 + t].b = top2 / bottom;
        }
    }
}

static DATA_TYPE
intersect_line_line(const Bounds* line1, const Bounds* line2)
{
    return (line1->b - line2->b) / (line2->a - line1->a);
}

static DATA_TYPE
dist_from_pole_squared(DATA_TYPE x, DATA_TYPE y)
{
    return x * x + y * y;
}

static DATA_TYPE
ray_length_until_intersect(DATA_TYPE theta, const Bounds* line)
{
    return line->b / (SIN_F(theta) - line->a * COS_F(theta));
}

static DATA_TYPE
max_safe_chroma_for_l(DATA_TYPE l)
{
    DATA_TYPE min_len_squared = DBL_MAX;
    Bounds bounds[6];
    int i;

    get_bounds(l, bounds);
    for(i = 0; i < 6; i++) {
        DATA_TYPE m1 = bounds[i].a;
        DATA_TYPE b1 = bounds[i].b;
        /* x where line intersects with perpendicular running though (0, 0) */
        Bounds line2 = { -1.0 / m1, 0.0 };
        DATA_TYPE x = intersect_line_line(&bounds[i], &line2);
        DATA_TYPE distance = dist_from_pole_squared(x, b1 + x * m1);

        if(distance < min_len_squared)
            min_len_squared = distance;
    }

    return SQRT_F(min_len_squared);
}

static DATA_TYPE
max_chroma_for_lh(DATA_TYPE l, DATA_TYPE h)
{
    DATA_TYPE min_len = DBL_MAX;
    DATA_TYPE hrad = h * 0.01745329251994329577; /* (2 * pi / 360) */
    Bounds bounds[6];
    int i;

    get_bounds(l, bounds);
    for(i = 0; i < 6; i++) {
        DATA_TYPE len = ray_length_until_intersect(hrad, &bounds[i]);

        if(len >= 0  &&  len < min_len)
            min_len = len;
    }
    return min_len;
}

static DATA_TYPE
dot_product(const Triplet* t1, const Triplet* t2)
{
    return (t1->a * t2->a + t1->b * t2->b + t1->c * t2->c);
}

/* Used for rgb conversions */
static DATA_TYPE
from_linear(DATA_TYPE c)
{
    if(c <= 0.0031308)
        return 12.92 * c;
    else
        return 1.055 * POW_F(c, 1.0 / 2.4) - 0.055;
}

static DATA_TYPE
to_linear(DATA_TYPE c)
{
    if (c > 0.04045)
        return POW_F((c + 0.055) / 1.055, 2.4);
    else
        return c / 12.92;
}

static void
xyz2rgb(Triplet* in_out)
{
    DATA_TYPE r = from_linear(dot_product(&m[0], in_out));
    DATA_TYPE g = from_linear(dot_product(&m[1], in_out));
    DATA_TYPE b = from_linear(dot_product(&m[2], in_out));
    in_out->a = r;
    in_out->b = g;
    in_out->c = b;
}

static void
rgb2xyz(Triplet* in_out)
{
    Triplet rgbl = { to_linear(in_out->a), to_linear(in_out->b), to_linear(in_out->c) };
    DATA_TYPE x = dot_product(&m_inv[0], &rgbl);
    DATA_TYPE y = dot_product(&m_inv[1], &rgbl);
    DATA_TYPE z = dot_product(&m_inv[2], &rgbl);
    in_out->a = x;
    in_out->b = y;
    in_out->c = z;
}

/* https://en.wikipedia.org/wiki/CIELUV
 * In these formulas, Yn refers to the reference white point. We are using
 * illuminant D65, so Yn (see refY in Maxima file) equals 1. The formula is
 * simplified accordingly.
 */
static DATA_TYPE
y2l(DATA_TYPE y)
{
    if(y <= epsilon)
        return y * kappa;
    else
        return 116.0 * CBRT_F(y) - 16.0;
}

static DATA_TYPE
l2y(DATA_TYPE l)
{
    if(l <= 8.0) {
        return l / kappa;
    } else {
        DATA_TYPE x = (l + 16.0) / 116.0;
        return (x * x * x);
    }
}

static void
xyz2luv(Triplet* in_out)
{
    DATA_TYPE var_u = (4.0 * in_out->a) / (in_out->a + (15.0 * in_out->b) + (3.0 * in_out->c));
    DATA_TYPE var_v = (9.0 * in_out->b) / (in_out->a + (15.0 * in_out->b) + (3.0 * in_out->c));
    DATA_TYPE l = y2l(in_out->b);
    DATA_TYPE u = 13.0 * l * (var_u - ref_u);
    DATA_TYPE v = 13.0 * l * (var_v - ref_v);

    in_out->a = l;
    if(l < 0.00000001) {
        in_out->b = 0.0;
        in_out->c = 0.0;
    } else {
        in_out->b = u;
        in_out->c = v;
    }
}

static void
luv2xyz(Triplet* in_out)
{
    if(in_out->a <= 0.00000001) {
        /* Black will create a divide-by-zero error. */
        in_out->a = 0.0;
        in_out->b = 0.0;
        in_out->c = 0.0;
        return;
    }

    DATA_TYPE var_u = in_out->b / (13.0 * in_out->a) + ref_u;
    DATA_TYPE var_v = in_out->c / (13.0 * in_out->a) + ref_v;
    DATA_TYPE y = l2y(in_out->a);
    DATA_TYPE x = -(9.0 * y * var_u) / ((var_u - 4.0) * var_v - var_u * var_v);
    DATA_TYPE z = (9.0 * y - (15.0 * var_v * y) - (var_v * x)) / (3.0 * var_v);
    in_out->a = x;
    in_out->b = y;
    in_out->c = z;
}

static void
luv2lch(Triplet* in_out)
{
    DATA_TYPE l = in_out->a;
    DATA_TYPE u = in_out->b;
    DATA_TYPE v = in_out->c;
    DATA_TYPE h;
    DATA_TYPE c = SQRT_F(u * u + v * v);

    /* Grays: disambiguate hue */
    if(c < 0.00000001) {
        h = 0;
    } else {
        h = ATAN2_F(v, u) * 57.29577951308232087680;  /* (180 / pi) */
        if(h < 0.0)
            h += 360.0;
    }

    in_out->a = l;
    in_out->b = c;
    in_out->c = h;
}

static void
lch2luv(Triplet* in_out)
{
    DATA_TYPE hrad = in_out->c * 0.01745329251994329577;  /* (pi / 180.0) */
    DATA_TYPE u = COS_F(hrad) * in_out->b;
    DATA_TYPE v = SIN_F(hrad) * in_out->b;

    in_out->b = u;
    in_out->c = v;
}

static void
hsluv2lch(Triplet* in_out)
{
    DATA_TYPE h = in_out->a;
    DATA_TYPE s = in_out->b;
    DATA_TYPE l = in_out->c;
    DATA_TYPE c;

    /* White and black: disambiguate chroma */
    if(l > 99.9999999 || l < 0.00000001)
        c = 0.0;
    else
        c = max_chroma_for_lh(l, h) / 100.0 * s;

    /* Grays: disambiguate hue */
    if (s < 0.00000001)
        h = 0.0;

    in_out->a = l;
    in_out->b = c;
    in_out->c = h;
}

static void
lch2hsluv(Triplet* in_out)
{
    DATA_TYPE l = in_out->a;
    DATA_TYPE c = in_out->b;
    DATA_TYPE h = in_out->c;
    DATA_TYPE s;

    /* White and black: disambiguate saturation */
    if(l > 99.9999999 || l < 0.00000001)
        s = 0.0;
    else
        s = c / max_chroma_for_lh(l, h) * 100.0;

    /* Grays: disambiguate hue */
    if (c < 0.00000001)
        h = 0.0;

    in_out->a = h;
    in_out->b = s;
    in_out->c = l;
}

static void
hpluv2lch(Triplet* in_out)
{
    DATA_TYPE h = in_out->a;
    DATA_TYPE s = in_out->b;
    DATA_TYPE l = in_out->c;
    DATA_TYPE c;

    /* White and black: disambiguate chroma */
    if(l > 99.9999999 || l < 0.00000001)
        c = 0.0;
    else
        c = max_safe_chroma_for_l(l) / 100.0 * s;

    /* Grays: disambiguate hue */
    if (s < 0.00000001)
        h = 0.0;

    in_out->a = l;
    in_out->b = c;
    in_out->c = h;
}

static void
lch2hpluv(Triplet* in_out)
{
    DATA_TYPE l = in_out->a;
    DATA_TYPE c = in_out->b;
    DATA_TYPE h = in_out->c;
    DATA_TYPE s;

    /* White and black: disambiguate saturation */
    if (l > 99.9999999 || l < 0.00000001)
        s = 0.0;
    else
        s = c / max_safe_chroma_for_l(l) * 100.0;

    /* Grays: disambiguate hue */
    if (c < 0.00000001)
        h = 0.0;

    in_out->a = h;
    in_out->b = s;
    in_out->c = l;
}



void
hsluv2rgb(DATA_TYPE h, DATA_TYPE s, DATA_TYPE l, DATA_TYPE* pr, DATA_TYPE* pg, DATA_TYPE* pb)
{
    Triplet tmp = { h, s, l };

    hsluv2lch(&tmp);
    lch2luv(&tmp);
    luv2xyz(&tmp);
    xyz2rgb(&tmp);

    *pr = CLAMP(tmp.a, 0.0, 1.0);
    *pg = CLAMP(tmp.b, 0.0, 1.0);
    *pb = CLAMP(tmp.c, 0.0, 1.0);
}

void
hpluv2rgb(DATA_TYPE h, DATA_TYPE s, DATA_TYPE l, DATA_TYPE* pr, DATA_TYPE* pg, DATA_TYPE* pb)
{
    Triplet tmp = { h, s, l };

    hpluv2lch(&tmp);
    lch2luv(&tmp);
    luv2xyz(&tmp);
    xyz2rgb(&tmp);

    *pr = CLAMP(tmp.a, 0.0, 1.0);
    *pg = CLAMP(tmp.b, 0.0, 1.0);
    *pb = CLAMP(tmp.c, 0.0, 1.0);
}

void
rgb2hsluv(DATA_TYPE r, DATA_TYPE g, DATA_TYPE b, DATA_TYPE* ph, DATA_TYPE* ps, DATA_TYPE* pl)
{
    Triplet tmp = { r, g, b };

    rgb2xyz(&tmp);
    xyz2luv(&tmp);
    luv2lch(&tmp);
    lch2hsluv(&tmp);

    *ph = CLAMP(tmp.a, 0.0, 360.0);
    *ps = CLAMP(tmp.b, 0.0, 100.0);
    *pl = CLAMP(tmp.c, 0.0, 100.0);
}

int
rgb2hpluv(DATA_TYPE r, DATA_TYPE g, DATA_TYPE b, DATA_TYPE* ph, DATA_TYPE* ps, DATA_TYPE* pl)
{
    Triplet tmp = { r, g, b };

    rgb2xyz(&tmp);
    xyz2luv(&tmp);
    luv2lch(&tmp);
    lch2hpluv(&tmp);

    *ph = CLAMP(tmp.a, 0.0, 360.0);
    /* Do NOT clamp the saturation. Application may want to have an idea
     * how much off the valid range the given RGB color is. */
    *ps = tmp.b;
    *pl = CLAMP(tmp.c, 0.0, 100.0);

    return (0.0 <= tmp.b  &&  tmp.b <= 100.0) ? 0 : -1;
}
