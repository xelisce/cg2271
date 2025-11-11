/* ============================================
   MELODY & DURATIONS
   ============================================ */

/* "Mary Had a Little Lamb" melody */
const uint16_t melody[] = {
    // "Mary had a little lamb"
    NOTE_E4, NOTE_D4, NOTE_C4, NOTE_D4, NOTE_E4, NOTE_E4, NOTE_E4,
    // "Little lamb, little lamb"
    NOTE_D4, NOTE_D4, NOTE_D4, NOTE_REST, NOTE_E4, NOTE_G4, NOTE_G4,
    // "Mary had a little lamb"
    NOTE_E4, NOTE_D4, NOTE_C4, NOTE_D4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4,
    // "Its fleece was white as snow"
    NOTE_D4, NOTE_D4, NOTE_E4, NOTE_D4, NOTE_C4};

const uint16_t durations[] = {
    // "Mary had a little lamb"
    QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, HALF, QUARTER,
    // "Little lamb, little lamb"
    QUARTER, QUARTER, HALF, QUARTER, QUARTER, QUARTER, HALF, QUARTER,
    // "Mary had a little lamb"
    QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER,
    // "Its fleece was white as snow"
    QUARTER, QUARTER, QUARTER, QUARTER, WHOLE};

/* "Twinkle Twinkle Little Star" melody */
const uint16_t melody[] = {
    NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4,
    NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4,
    NOTE_G4, NOTE_G4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4,
    NOTE_G4, NOTE_G4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4,
    NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4,
    NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4};

const uint16_t durations[] = {
    QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, HALF,
    QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, HALF,
    QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, HALF,
    QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, HALF,
    QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, HALF,
    QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, HALF};