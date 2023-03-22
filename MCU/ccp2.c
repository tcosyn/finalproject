#include <xc.h>

extern uint8_t right_freq_1, right_freq_2;
static volatile uint16_t prev_value;

void CCP2_Init(void) {
	CCP2PPS = 0x11;   //RC1->CCP2:CCP2; 
	CCP2CON = 0x85;      
	CCPTMRS0bits.C2TSEL = 0x1;
    PIR7bits.CCP2IF = 0;
    prev_value = 0;
}

void CCP2_ISR(void) {
    PIR7bits.CCP2IF = 0;
    uint16_t temp = prev_value;
    uint8_t freq_1, freq_2;
    freq_1 = CCPR2H;
    freq_2 = CCPR2L;
    uint16_t freq_comb = (freq_1 << 8) | freq_2;
    if (freq_comb < prev_value) {
        prev_value = freq_comb;
        uint16_t result = (0xFFFF - temp) + freq_comb;
        right_freq_1 = (result & 0xFF00) >> 8;
        right_freq_2 = (result & 0x00FF);
    } else {
        prev_value = freq_comb;
        uint16_t result = freq_comb - temp;
        right_freq_1 = (result & 0xFF00) >> 8;
        right_freq_2 = result & 0x00FF;
    }
}
