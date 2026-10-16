

#include "p2_motion.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>

int main(void) {
    P2_Motion state;
    uint32_t session, previous_session = 0U, ms;
    unsigned int acc_fresh, mag_fresh;
    P2_Vector acc, mag;
    bool have_session = false;
    int fields;
    P2_MotionInit(&state);
    while ((fields = scanf("%" SCNu32 " %" SCNu32 " %u %u %f %f %f %f %f %f",
                           &session, &ms, &acc_fresh, &mag_fresh,
                           &acc.x, &acc.y, &acc.z, &mag.x, &mag.y, &mag.z)) == 10) {
        if (!have_session || session != previous_session) {
            P2_MotionInit(&state);
            previous_session = session;
            have_session = true;
        }
        if (acc_fresh != 0U) {
            P2_MotionAcceleration(&state, acc, ms);
        } else {

            P2_Vector invalid_acc = {NAN, NAN, NAN};
            P2_MotionAcceleration(&state, invalid_acc, ms);
            state.heading_valid = false;
        }
        if (acc_fresh != 0U && mag_fresh != 0U) {
            (void)P2_MotionHeading(&state, mag);
        } else {
            state.heading_valid = false;
        }
        printf("%u %.9g %" PRIu32 "\n", state.heading_valid ? 1U : 0U,
               state.heading_valid ? (double)state.heading_deg : -1.0,
               state.steps);
    }
    if (fields != EOF) {
        fprintf(stderr, "Malformed replay input (expected 10 fields).\n");
        return 2;
    }
    return ferror(stdin) || ferror(stdout) ? 1 : 0;
}
