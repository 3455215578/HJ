

#include "main.h"
#include "filter.h"


/************ һ�׿������˲�*************/

/**
  * @name   kalmanCreate
  * @brief  ����һ���������˲���
  * @param  p:  �˲���
  *         T_Q:ϵͳ����Э����
  *         T_R:��������Э����
  *
  * @retval none
  * @attention R�̶�,QԽ�����Խ���β���ֵ��Q�������ֻ�ò���ֵ
  *		       Q�̶�,RԽ�����Խ����Ԥ��ֵ��R�������ֻ��Ԥ��ֵ
  */
void first_Kalman_Create(first_kalman_filter_t *p, float T_Q, float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}


/**
  * @name   KalmanFilter
  * @brief  �������˲���
  * @param  p:  �˲���
  *         dat:���˲�����
  * @retval �˲��������
  * @attention Z(k)��ϵͳ����,������ֵ   X(k|k)�ǿ������˲����ֵ,���������
  *            A=1 B=0 H=1 I=1  W(K)  V(k)�Ǹ�˹������,�����ڲ���ֵ����,���Բ��ù�
  *            �����ǿ�������5�����Ĺ�ʽ
  *            һ��H'��Ϊ������,����Ϊת�þ���
  */

float first_Kalman_Filter(first_kalman_filter_t* p, float dat)
{
    p->X_mid =p->A*p->X_last;                     //�ٶȶ�Ӧ��ʽ(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)     ״̬����
    p->P_mid = p->A*p->P_last+p->Q;               //�ٶȶ�Ӧ��ʽ(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q            �۲ⷽ��
    p->kg = p->P_mid/(p->P_mid+p->R);             //�ٶȶ�Ӧ��ʽ(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)   ���¿���������
    p->X_now = p->X_mid + p->kg*(dat-p->X_mid);   //�ٶȶ�Ӧ��ʽ(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))  ��������ֵ
    p->P_now = (1-p->kg)*p->P_mid;                //�ٶȶ�Ӧ��ʽ(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)           ���º������Э����
    p->P_last = p->P_now;                         //״̬����
    p->X_last = p->X_now;
    return p->X_now;							  //���Ԥ����x(k|k)
}

/************ һ�׿������˲�*************/



/************ ���׿������˲� *************/

float matrix_value1;
float matrix_value2;

void kalman_filter_init(second_kalman_filter_t *F, second_kalman_filter_init_t *I)
{
    mat_init(&F->xhat,2,1,(float *)I->xhat_data);
    mat_init(&F->xhatminus,2,1,(float *)I->xhatminus_data);
    mat_init(&F->z,2,1,(float *)I->z_data);
    mat_init(&F->A,2,2,(float *)I->A_data);
    mat_init(&F->H,2,2,(float *)I->H_data);
    mat_init(&F->Q,2,2,(float *)I->Q_data);
    mat_init(&F->R,2,2,(float *)I->R_data);
    mat_init(&F->P,2,2,(float *)I->P_data);
    mat_init(&F->Pminus,2,2,(float *)I->Pminus_data);
    mat_init(&F->K,2,2,(float *)I->K_data);
    mat_init(&F->AT,2,2,(float *)I->AT_data);
    mat_trans(&F->A, &F->AT);
    mat_init(&F->HT,2,2,(float *)I->HT_data);
    mat_trans(&F->H, &F->HT);
//  matrix_value2 = F->A.pData[1];
}


// xhatminus==x(k|k-1)  xhat==X(k-1|k-1)
// Pminus==p(k|k-1)     P==p(k-1|k-1)    AT==A'
// HT==H'   K==kg(k)    I=1
//

/**
  *@param �����������ṹ��
	�о��߽�Ҳ�ǿ�����ô�㵹��
  *@param �Ƕ�
  *@param �ٶ�
*/
float *kalman_filter_calc(second_kalman_filter_t *F, float signal1, float signal2)
{
    float TEMP_data[4] = {0, 0, 0, 0};
    float TEMP_data21[2] = {0, 0};
    mat TEMP,TEMP21;

    mat_init(&TEMP,2,2,(float *)TEMP_data);//
    mat_init(&TEMP21,2,1,(float *)TEMP_data21);//

    //��ȡ����ֵ
    F->z.pData[0] = signal1;//z(k)
    F->z.pData[1] = signal2;//z(k)

    //����˻��ǵ�һ�������˵ڶ�������Ȼ�󽫽���ŵ���������������
    //1. xhat'(k)= A xhat(k-1)  xhat->ָ��^��x
    //��һ����������һʱ��Xhat���Ž�Ԥ�⵱ǰxhat
    mat_mult(&F->A, &F->xhat, &F->xhatminus);//  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)

    //2. P'(k) = A P(k-1) AT + Q
    //�ڶ������ɹ�ȥ��Э���������㵱ǰxhat��Э�������
    mat_mult(&F->A, &F->P, &F->Pminus);//   p(k|k-1) = A*p(k-1|k-1)*A'+Q
    mat_mult(&F->Pminus, &F->AT, &TEMP);//  p(k|k-1) = A*p(k-1|k-1)*A'+Q
    mat_add(&TEMP, &F->Q, &F->Pminus);//    p(k|k-1) = A*p(k-1|k-1)*A'+Q

    //3. K(k) = P'(k) HT / (H P'(k) HT + R)
    //�����������ݵ�ǰЭ���������㿨��������
    //��һ���������һ�����ⶼ����K���������������һ������
    mat_mult(&F->H, &F->Pminus, &F->K);//  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    mat_mult(&F->K, &F->HT, &TEMP);//      kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    mat_add(&TEMP, &F->R, &F->K);//        kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)

    mat_inv(&F->K, &F->P);	//	����ʽ��ĸȡ���ٳ�
    mat_mult(&F->Pminus, &F->HT, &TEMP);//
    mat_mult(&TEMP, &F->P, &F->K);//

    //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
    //���Ĳ�������QR���ۺ�Ԥ�����Ͳ���ֵ�����㵱ǰxhat���Ž�
    mat_mult(&F->H, &F->xhatminus, &TEMP21);//      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    mat_sub(&F->z, &TEMP21, &F->xhat);//            x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    mat_mult(&F->K, &F->xhat, &TEMP21);//           x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    mat_add(&F->xhatminus, &TEMP21, &F->xhat);//    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

    //5. P(k) = (1-K(k)H)P'(k)
    //���岽�����㵱ǰ���Ž��Э�������
    mat_mult(&F->K, &F->H, &F->P);//            p(k|k) = (I-kg(k)*H)*P(k|k-1)
    mat_sub(&F->Q, &F->P, &TEMP);//
    mat_mult(&TEMP, &F->Pminus, &F->P);

    //��ȡ���׿������ļ�����
    matrix_value1 = F->xhat.pData[0];
    matrix_value2 = F->xhat.pData[1];

    F->filtered_value[0] = F->xhat.pData[0];
    F->filtered_value[1] = F->xhat.pData[1];
    return F->filtered_value;
}



//�ο�����RPս�ӵ��˲��㷨

/************ ������ֵ�˲� *************/

/**
  * @brief    average_init
  * @note    �����˲�����ʼ�������ó���
  * @param  None
  * @retval None
  */
void average_init(moving_Average_Filter *Aver, uint8_t length)
{
    uint16_t i;

    for(i = 0; i<MAF_MaxSize; i++)
        Aver->num[i] = 0;

    if(length >MAF_MaxSize)
    {
        length = MAF_MaxSize;
    }

    Aver->length = length;
    Aver->pot = 0;
    Aver->aver_num = 0;
    Aver->total = 0;
}

/**
  * @brief    average_clear
  * @note    �����˲������
  * @param  None
  * @retval None
  */
void average_clear(moving_Average_Filter *Aver)
{
    uint16_t i;

    for(i = 0; i<MAF_MaxSize; i++)
        Aver->num[i] = 0;

    Aver->pot = 0;
    Aver->aver_num = 0;
    Aver->total = 0;
}

/**
  * @brief    average_fill
  * @note    �����˲������ĳ��ֵ
  * @param  None
  * @retval None
  * @author  RobotPilots
  */
void average_fill(moving_Average_Filter *Aver, float temp)
{
    uint16_t i;

    for(i = 0; i<(Aver->length); i++)
        Aver->num[i] = temp;

    Aver->pot = 0;
    Aver->aver_num = temp;
    Aver->total = temp*(Aver->length);
}

/**
  * @brief    average_add
  * @note    ����ƽ���˲���������У��Ƚ��ȳ�
  * @param  None
  * @retval None
  * @author  RobotPilots
  */
void average_add(moving_Average_Filter *Aver, float add_data)
{

    Aver->total -=  Aver->num[Aver->pot];
    Aver->total += add_data;

    Aver->num[Aver->pot] = add_data;

    Aver->aver_num = (Aver->total)/(Aver->length);
    Aver->pot++;

    if(Aver->pot == Aver->length)
    {
        Aver->pot = 0;
    }

}

/**
  * @brief    average_get
  * @note    ��ȡ��ǰpre�ε����ݣ�����������鳤����ȡ��¼�����������
  * @param  None
  * @retval None
  * @author  RobotPilots
  */
float average_get(moving_Average_Filter *Aver, uint16_t pre)
{
    float member;
    uint8_t temp;

    if(Aver->pot != 0)
    {
        temp = Aver->pot-1;
    }
    else
    {
        temp = Aver->length-1;
    }

    if(pre>Aver->length)
        pre = pre % Aver->length;

    if(pre>temp)
    {
        pre = Aver->length+temp-pre;
    }
    else
    {
        pre = temp-pre;
    }

    member = Aver->num[pre];

    return member;
}


//���׵�ͨ�˲�
void SetCutoffFreq(second_lowPass_filter *lf,float sample_freq, float cutoff_freq)
{
    float fr =0;
    float ohm =0;
    float c =0;

    fr= sample_freq/cutoff_freq;
    ohm=tanf(PI/fr);
    c=1.0f+2.0f*cosf(PI/4.0f)*ohm + ohm*ohm;

    lf->_cutoff_freq1 = cutoff_freq;

    if (lf->_cutoff_freq1 > 0.0f)
    {
        lf->_b01 = ohm*ohm/c;
        lf->_b11 = 2.0f*lf->_b01;
        lf->_b21 = lf->_b01;
        lf->_a11 = 2.0f*(ohm*ohm-1.0f)/c;
        lf->_a21 = (1.0f-2.0f*cosf(PI/4.0f)*ohm+ohm*ohm)/c;
    }
}

float Apply(second_lowPass_filter *LF,float sample)
{
    float delay_element_0 = 0, output=0;
    if (LF->_cutoff_freq1 <= 0.0f)
    {
        // no filtering
        return sample;
    }
    else
    {
        delay_element_0 = sample - LF->_delay_element_11 * LF->_a11 - LF->_delay_element_21 * LF->_a21;
        // do the filtering
        if (isnan(delay_element_0) || isinf(delay_element_0)) {
            // don't allow bad values to propogate via the filter
            delay_element_0 = sample;
        }
        output = delay_element_0 * LF->_b01 + LF->_delay_element_11 *LF->_b11 + LF->_delay_element_21 * LF->_b21;

        LF->_delay_element_21 = LF->_delay_element_11;
        LF->_delay_element_11 = delay_element_0;

        // return the value.  Should be no need to check limits
        return output;
    }
}