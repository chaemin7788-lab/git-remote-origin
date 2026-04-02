#include "Sequence.h"

// 열거형(ENUM)을 사용하여 단계 정의

// DC REALY SEQ
enum AutomationStepDCR {
	STEP_DCR_ON,
	STEP_DCR_OFF,
	STEP_DCR_ON1,
	STEP_DCR_ON2,
	STEP_DCR_OFF1,
	STEP_DCR_OFF2,
    STEP_DCR_DONE
};
uint8_t DcrSeqCount = STEP_DCR_DONE;

// MOP SEQ
enum AutomationStepVCB {
	STEP_VCB_ON,
	STEP_VCB_OFF,
	STEP_VCB_ON1,
	STEP_VCB_ON2,
	STEP_VCB_OFF1,
	STEP_VCB_OFF2,
    STEP_VCB_DONE
};
uint8_t VcbSeqCount = STEP_VCB_DONE;

// MOP SEQ
enum AutomationStepMOP {
	STEP_MOP_ON,
	STEP_MOP_OFF,
	STEP_MOP_ON1,
	STEP_MOP_ON2,
	STEP_MOP_OFF1,
	STEP_MOP_OFF2,
	STEP_MOP_DONE
};
uint8_t MopSeqCount = STEP_MOP_DONE;

void MopSequence(enum AutomationStepMOP step);
void VcbSequence(enum AutomationStepVCB step);
void DcrSequence(enum AutomationStepDCR step);

// 시퀀스를 수행하는 함수
void MopSequence(enum AutomationStepMOP step) {
    switch (step) {
        case STEP_MOP_ON:
            printf("MOP ON...\r\n");
            RELAY_MOP_ON_SIG_OFF;
            RELAY_MOP_OFF_SIG_OFF;
            MopSeqCount=STEP_MOP_ON1;
            break;
        case STEP_MOP_OFF:
            printf("MOP OFF...\r\n");
            RELAY_MOP_ON_SIG_OFF;
            RELAY_MOP_OFF_SIG_OFF;
            MopSeqCount=STEP_MOP_OFF1;
            break;
        case STEP_MOP_ON1:
            printf("MOP ON STEP1 \r\n");
            RELAY_MOP_ON_SIG_ON;
            MopSeqCount=STEP_MOP_ON2;
            break;
        case STEP_MOP_ON2:
            printf("MOP ON DONE! \r\n");
            RELAY_MOP_ON_SIG_OFF;
            MopSeqCount=STEP_MOP_DONE;
            break;
        case STEP_MOP_OFF1:
            printf("MOP OFF STEP1 \r\n");
            RELAY_MOP_OFF_SIG_ON;
            MopSeqCount=STEP_MOP_OFF2;
            break;
        case STEP_MOP_OFF2:
        	printf("MOP ON DONE! \r\n");
        	RELAY_MOP_OFF_SIG_OFF;
            MopSeqCount=STEP_MOP_DONE;
            break;
        case STEP_MOP_DONE:
        	;           // DO NOTING
            break;
        default:
            ;
            break;
    }
}

// 시퀀스를 수행하는 함수
void VcbSequence(enum AutomationStepVCB step) {
    switch (step) {
        case STEP_VCB_ON:
            printf("VCB ON...\r\n");
            RELAY_VCP_ON_SIG_OFF;
            RELAY_VCP_OFF_SIG_OFF;
            VcbSeqCount=STEP_VCB_ON1;
            break;
        case STEP_VCB_OFF:
            printf("VCB OFF...\r\n");
            RELAY_VCP_ON_SIG_OFF;
            RELAY_VCP_OFF_SIG_OFF;
            VcbSeqCount=STEP_VCB_OFF1;
            break;
        case STEP_VCB_ON1:
            printf("VCB ON STEP1 \r\n");
            RELAY_VCP_ON_SIG_ON;
            VcbSeqCount=STEP_VCB_ON2;
            break;
        case STEP_VCB_ON2:
            printf("VCB ON DONE! \r\n");
            RELAY_VCP_ON_SIG_OFF;
            VcbSeqCount=STEP_VCB_DONE;
            break;
        case STEP_VCB_OFF1:
            printf("VCB OFF STEP1 \r\n");
            RELAY_VCP_OFF_SIG_ON;
            VcbSeqCount=STEP_VCB_OFF2;
            break;
        case STEP_VCB_OFF2:
            printf("VCB OFF DONE! \r\n");
            RELAY_VCP_OFF_SIG_OFF;
        	VcbSeqCount=STEP_VCB_DONE;
            break;
        case STEP_VCB_DONE:
        	;           // DO NOTING
            break;
        default:
            ;
            break;
    }
}


// 시퀀스를 수행하는 함수
// 프리챠저 기동
void DcrSequence(enum AutomationStepDCR step) {
    switch (step) {
        case STEP_DCR_ON:
            printf("DCR ON...\r\n");
            DcrSeqCount=STEP_DCR_ON1;
            break;
        case STEP_DCR_OFF:
            printf("DCR OFF... \r\n");
            DcrSeqCount=STEP_DCR_OFF1;
            break;
        case STEP_DCR_ON1:
            printf("Pre-Charger ON! \r\n");
            RELAY_FRC_REALY1_SIG_ON;
            osDelay(100);
            RELAY_FRC_REALY2_SIG_ON;
            osDelay(1000);
            RELAY_FRC_REALY3_SIG_ON;
            DcrSeqCount=STEP_DCR_DONE;
            break;
        case STEP_DCR_OFF1:
            printf("Pre-Charger OFF! \r\n");
            RELAY_FRC_REALY1_SIG_OFF;
            RELAY_FRC_REALY2_SIG_OFF;
            RELAY_FRC_REALY3_SIG_OFF;
            DcrSeqCount=STEP_DCR_DONE;
            break;
        case STEP_DCR_DONE:
        	;           // DO NOTING
            break;
        default:
            ;
            break;
    }
}

// 모든 신호를 초기화 한다.
void SequenceInit()
{
	// 전체 RELAY SIGNAL을 OFF 상태로 초기화 한다.
    RELAY_MOP_ON_SIG_OFF;
    RELAY_MOP_OFF_SIG_OFF;

    RELAY_VCP_ON_SIG_OFF;
    RELAY_VCP_OFF_SIG_OFF;
    RELAY_VCP_OFF_SIG_MOTOR;
	RELAY_VCP_OFF_SIG_UVT;

    RELAY_FRC_REALY1_SIG_OFF;
    RELAY_FRC_REALY2_SIG_OFF;
    RELAY_FRC_REALY3_SIG_OFF;
    //RELAY_FRC_REALY4_SIG_OFF;

}

// 트리거 샌택
void SequenceTriger(uint8_t index)
{
    switch (index) {
        case 1:
        	if((MopSeqCount == STEP_MOP_DONE) && (READ_MOP_READ_ON_SIG != FALSE)) MopSeqCount = STEP_MOP_ON;
        	break;
        case 2:
        	if((MopSeqCount == STEP_MOP_DONE) && (READ_MOP_READ_ON_SIG != TRUE)) MopSeqCount = STEP_MOP_OFF;
        	break;
        case 3:
        	if((VcbSeqCount == STEP_VCB_DONE) && (READ_VCP_READ_ON_SIG != FALSE)) VcbSeqCount = STEP_VCB_ON;
        	break;
        case 4:
        	if((VcbSeqCount == STEP_VCB_DONE) && (READ_VCP_READ_ON_SIG != TRUE)) VcbSeqCount = STEP_VCB_OFF;
        	break;
        case 5:
        	if(DcrSeqCount == STEP_DCR_DONE) DcrSeqCount = STEP_DCR_ON;
        	break;
        case 6:
        	if(DcrSeqCount == STEP_DCR_DONE) DcrSeqCount = STEP_DCR_OFF;
        	break;
        default:
        	break;
    }
}

// 시퀀스 실행 1초 간격
void SequenceMain()
{
	DcrSequence(DcrSeqCount);
	MopSequence(MopSeqCount);
	VcbSequence(VcbSeqCount);
}


void enable_Seq(uint16_t index, unsigned char state)
{
    switch(index)
    {
		case 5:
			;
			break;
		case 6:
			;
			break;
		case 7:
			;
			break;
		case 8:
			;
			break;
		case 9:
			if(state == 1){RELAY_DC_REALY2_SIG_ON;RELAY_DC_REALY3_SIG_ON;}
			else {RELAY_DC_REALY2_SIG_OFF;RELAY_DC_REALY3_SIG_OFF;}
			break;
		case 10:
			if(state == 1){RELAY_DC_REALY1_SIG_ON;}
			else {RELAY_DC_REALY1_SIG_OFF;}
			break;
		case 11:
			if(state == 1)SequenceTriger(6);
			break;
		case 12:
			if(state == 1)SequenceTriger(5);
			break;
		case 13:
			if(state == 1)SequenceTriger(4);
			break;
    	case 14:
			if(state == 1)SequenceTriger(3);
			break;
        case 15:
            if(state == 1)SequenceTriger(2);
            break;
        case 16:
            if(state == 1)SequenceTriger(1);
            break;
    }

}

// 모드버스 데이터를 읽어와서 실행한다.
void check_Seq(uint16_t fan_map)
{
	uint16_t i;
	uint16_t bit;
	unsigned char state;

    for(i=0; i < 16; i++) {
        bit = (fan_map&(0x01<<i));
        state = ((bit != 0)? 1 : 0);
        enable_Seq( (16-i), state);
    }
}
