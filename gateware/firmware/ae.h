/* Streaming histogram metering and exposure control in 1/16-stop units.
   The ISP counts 512000 RAW10 pixel pairs per complete camera frame. */
#include <stdint.h>

typedef struct {
    uint32_t count;
    union { struct { uint32_t p50, p90, p99; }; uint32_t percentile[3]; };
    uint32_t clipped;
} AeHistogram;
typedef struct {
    union { struct { int32_t p50, p90, p99; }; int32_t percentile[3]; };
    int32_t exposure; uint32_t valid;
} AeHistory;

static void ae_bin(AeHistogram *h, uint32_t bin, uint32_t n) {
    /* The last bin entered before reaching a rank contains that percentile.
       This also handles empty bins and exact rank boundaries. */
    if (h->count<256000) h->p50=bin;
    if (h->count<460800) h->p90=bin;
    if (h->count<506880) h->p99=bin;
    h->count+=n;
    if (bin>=508) h->clipped+=n;
}

/* Integer log2, rounded down to 1/16 stop; zero is treated as one bin. */
static int32_t ae_log2(uint32_t value) {
    /* Exact transition points of the Q8 iterative-squaring fractional log,
       relative to 256. Storing its 15 boundaries replaces four multiplies. */
    static const uint8_t boundary[]={12,24,36,49,63,77,92,107,123,140,157,176,195,215,236};
    int32_t result=128;
    if (!value) value=1;
    while (value<256) { value<<=1; result-=16; }
    while (value>=512) { value>>=1; result+=16; }
    uint32_t fraction=0;
    while (fraction<15 && value>=256u+boundary[fraction]) ++fraction;
    return result+(int32_t)fraction;
}

static int32_t ae_error(AeHistory *history, const AeHistogram *a, const AeHistogram *b,
                        uint32_t energy, int32_t quarters, int32_t scratch[6]) {
    int32_t *r=scratch; uint32_t *p=(uint32_t *)(scratch+3);
    int32_t exposure=ae_log2(energy);
    for (uint32_t i=0;i<3;i++) {
        p[i]=a->percentile[i]>b->percentile[i] ? a->percentile[i] : b->percentile[i];
        r[i]=ae_log2(p[i])-exposure;
    }
    int32_t change=0;
    uint32_t hold=0;
    /* Keep scene brightness in log units with exposure removed. Once EV
       compensation clips a percentile, its measured value is censored.
       Predict it from the last unclipped observation, tracking illumination
       changes through the highest still-usable percentile (P90, then P50). */
    if (history->valid) {
        /* Sensor offsets and gain response are not perfectly proportional.
           Infer lighting drift only at a fixed exposure; otherwise our own
           adjustment can look like dimming and amplify the requested EV. */
        if (exposure==history->exposure) {
            if (p[1]>=4 && p[1]<508) change=r[1]-history->p90;
            else if (p[0]>=4 && p[0]<508) change=r[0]-history->p50;
            /* A noisy shadow percentile must not make the inferred clipped
               highlights wander. Retain the anchor until a quarter-stop
               change accumulates, so slow lighting changes still register. */
            if (p[2]>=508 && change>-4 && change<4) { change=0; hold=1; }
        }
        if (p[0]<4 || p[0]>=508) r[0]=history->p50+change;
        for (uint32_t i=1;i<3;i++)
            if (p[i]>=508 && history->percentile[i]+change>r[i]) r[i]=history->percentile[i]+change;
    }
    for (uint32_t i=0;i<3;i++) if (!hold || i==2) history->percentile[i]=r[i];
    history->exposure=exposure;
    if (p[2]<508 && p[1]>=4) history->valid=1;
    /* Establish the automatic baseline from broad bright tones at 75% RAW
       and a 95% P99 ceiling, then apply the user's EV offset to that baseline.
       Positive EV is allowed to clip highlights instead of being cancelled. */
    int32_t error=137-r[1]-exposure; /* ae_log2(384) */
    int32_t ceiling=142-r[2]-exposure; /* ae_log2(486) */
    if (error>ceiling) error=ceiling;
    error+=quarters*4;
    /* Clipping hides how far overexposed the scene is: back off at least
       half a stop per update until the clipped fraction is at most 1%. */
    if ((quarters<=0 || !history->valid) &&
            (a->clipped>5120 || b->clipped>5120) && error>-16) error=-16;
    return error;
}

static uint32_t ae_abs(int32_t value) { return value<0 ? -value : value; }

/* Return lines in bits 9:0 and gain in bits 17:10. No floating point. */
static uint32_t ae_settings(uint32_t lines, uint32_t gain, int32_t error, int32_t exposure) {
    static const uint16_t exp2_q12[16]={4096,4277,4467,4664,4871,5086,5312,5547,
                                       5793,6049,6317,6597,6889,7192,7512,7845};
    uint32_t unchanged=lines|(gain<<10);
    if (ae_abs(error)<2) return unchanged; /* 1/8-stop deadband */
    int32_t step=(int32_t)(ae_abs(error)>>1);
    if (error<0) step=-step;
    if (step>8) step=8;       /* brighten by at most 1/2 stop */
    if (step< -16) step=-16; /* darken by at most one stop */
    uint32_t factor=exp2_q12[(uint32_t)step&15];
    if (step<0) factor>>=1;
    uint32_t energy=(lines*gain*factor+2048)>>12;
    uint32_t next_lines=(energy+8)>>4, next_gain=16;
    if (!next_lines) next_lines=1;
    if (next_lines>885) {
        next_lines=885; next_gain=(energy+442)/885;
        if (next_gain>255) next_gain=255;
    }
    if (next_lines==lines && next_gain==gain) {
        if (error>0) { if (lines<885) ++next_lines; else if (gain<255) ++next_gain; }
        else { if (gain>16) --next_gain; else if (lines>1) --next_lines; }
    }
    /* At very short exposure a whole line is a large EV step. Only make
       that quantized move if it reduces error, avoiding a 1/2-line cycle. */
    int32_t actual=ae_log2(next_lines*next_gain)-exposure;
    if (ae_abs(error-actual)>=ae_abs(error)) return unchanged;
    return next_lines|(next_gain<<10);
}
