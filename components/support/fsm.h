#ifndef _FSM_H_
#define _FSM_H_
 
#include <stdint.h>
#include <stddef.h>
 
typedef struct FsmTable_s
{
    uint8_t event;                /* �����¼� */
    uint8_t CurState;             /* ��ǰ״̬ */
    void (*eventActFun)(void *);  /* �������� */
    uint8_t NextState;            /* ��ת״̬ */
}FsmTable_T;
 
typedef struct FSM_s
{
    FsmTable_T *FsmTable;         /* ״̬Ǩ�Ʊ� */
    uint8_t curState;             /* ״̬����ǰ״̬ */
    uint16_t stuMaxNum;            /* ״̬��״̬Ǩ������ */
}FSM_T;




 
/*********************************************************************************
ʹ�÷�����1.����FSM_T����
          2.����FsmTable_T��
          3.����FSM_Init()��ʼ����
          4.������ѯFSM_EventHandle()����״̬����
*********************************************************************************/
void FSM_Init(FSM_T *pFsm, FsmTable_T *pTable, uint16_t stuMaxNum, uint8_t curState);
void FSM_EventHandle(FSM_T *pFsm, uint8_t event, void *parm);
 
#endif

