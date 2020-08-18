
/*!
 ***************************************************************************
 * \file ratectl.c
 *
 * \brief
 *    Rate Control algorithm
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details) 
 *     - Siwei Ma <swma@jdl.ac.cn>
 *     - Zhengguo LI<ezgli@lit.a-star.edu.sg>
 *
 * \date
 *   16 Jan. 2003
 **************************************************************************
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "global.h"
#include "ratectl.h"


const double THETA=1.3636;
const int Switch=0;

int Iprev_bits=0;
int Pprev_bits=0;


/* rate control variables */
int Xp, Xb;
static int R,T_field;
static int Np, Nb, bits_topfield, Q;
long T,T1;
//HRD consideration
long UpperBound1, UpperBound2, LowerBound;
double InitialDelayOffset;
const double OMEGA=0.9;

double Wp,Wb; 
int TotalPFrame;
int DuantQp; 
int PDuantQp;
FILE *BitRate;
double DeltaP;

// Initiate rate control parameters ��ʼ�����ʿ��Ʊ���
void rc_init_seq()
{
   double L1,L2,L3,bpp;
   int qp;
   int i;
  
   Xp=0;
   Xb=0;
   
   bit_rate=input->bit_rate;//Ŀ������
   frame_rate = (float)(img->framerate *(input->successive_Bframe + 1)) / (float) (input->jumpd + 1);//֡�� ??��ô���
   PreviousBit_Rate=bit_rate;//��������
   
   /*compute the total number of MBs in a frame*/
   
   img->Frame_Total_Number_MB=img->height*img->width/256;//16*16=256 ���к����
   if(input->basicunit > img->Frame_Total_Number_MB)
     input->basicunit = img->Frame_Total_Number_MB;//ÿ��������Ԫ�ĺ���������ܳ����ܺ����
   if(input->basicunit < img->Frame_Total_Number_MB)
     TotalNumberofBasicUnit = img->Frame_Total_Number_MB/input->basicunit;//������Ԫ����
   
   MINVALUE=4.0;
   /*initialize the parameters of fluid flow traffic model*/
   
   BufferSize=bit_rate*2.56;//��������С
   CurrentBufferFullness=0;//��ǰ�������ĳ�ӯ�̶�����
   GOPTargetBufferLevel=CurrentBufferFullness;//??
   /*HRD consideration*/
   InitialDelayOffset=BufferSize*0.8;
   
   /*initialize the previous window size*/
   m_windowSize=0;
   MADm_windowSize=0;
   img->NumberofCodedBFrame=0;//�Ѿ������B֡���¼���
   img->NumberofCodedPFrame=0;//�Ѿ������P֡���¼���
   img->NumberofGOP=0;
   /*remaining # of bits in GOP */
   R = 0;//��ǰGOP��ʣ��ı�����
   /*control parameter */
   if(input->successive_Bframe>0)//��B֡ P֡ PBB..BBP
   {
     GAMMAP=0.25;//����ο�ͼ��ı�����
     BETAP=0.9;
   }
   else//ֻ��P֡IPPPPI
   {
     GAMMAP=0.5;
     BETAP=0.5;
   }
   
   /*quadratic ƽ�� rate-distortion model*/
   PPreHeader=0;
   
   Pm_X1=bit_rate*1.0;//RDģ�Ͳ���
   Pm_X2=0.0;
   /* linear ���� prediction model for P picture*/
   PMADPictureC1=1.0;//��D(ʧ���)������Ԥ��
   PMADPictureC2=0.0;
   
   for(i=0;i<20;i++)
   {
     Pm_rgQp[i]=0;
     Pm_rgRp[i]=0.0;
     PPictureMAD[i]=0.0;
   }
   PPictureMAD[20]=0.0;
   
   //Define the largest variation of quantization parameters
   PDuantQp=2;//�����������ı仯��
   
   /*basic unit ������Ԫ layer rate control*/
    PAveHeaderBits1=0;
    PAveHeaderBits3=0;  
    if(TotalNumberofBasicUnit>=9)//������ͼ��û�п���bits�����������ƫ��QP�ĳ�ʼֵ ���ϵ��ж���8
      DDquant=1;
    else
      DDquant=2;
    
    MBPerRow=input->img_width/16;//ÿһ�еĺ����
    
    /*adaptive field/frame coding*/
    img->FieldControl=0;
    
    RC_MAX_QUANT = 51;  // clipping �޷� QP�����ֵ
    RC_MIN_QUANT = 0;//clippin
    
    /*compute thei initial QP*/
	//���ʳ���֡�ʾ���ÿһ��ͼ������bits �ٳ���ͼ������ �õ�ͼ����ƽ��ÿһ�����ط����bits
    bpp = 1.0*bit_rate /(frame_rate*img->width*img->height);
    if (img->width == 176) //QCIFͼ��
    {
      L1 = 0.1;
      L2 = 0.3;
      L3 = 0.6;
    }else if (img->width == 352)//CIFͼ��
    {
      L1 = 0.2;
      L2 = 0.6;
      L3 = 1.2;
    }else //����ͼ��
    {
      L1 = 0.6;
      L2 = 1.4;
      L3 = 2.4;
    }
    
    if (input->SeinitialQP==0)//�Ե�һ��GOP�ĵ�һ��IDR֡QP����
    {
      if(bpp<= L1)//���ÿ�����ط����bits���� ��ʼ���ýϴ��������� QP
        qp = 35;
      else
        if(bpp<=L2)//bppԽС �����QPԽС
          qp = 25;
        else
          if(bpp<=L3)
            qp  = 20;
          else
            qp =10;
          input->SeinitialQP = qp;
    }
}

// Initiate one GOP  ��ʼ��һ��GOP
void rc_init_GOP(int np, int nb)
{
  Boolean Overum=FALSE;//Ĭ��û�г���Ԥ��
  int OverBits;
  int OverDuantQp;
  int AllocatedBits;
  int GOPDquant;

  /*check if the last GOP over uses its budget. If yes, the initial QP of the I frame in 
 the coming  GOP will be increased.*/

  if(R<0)//��һ��GOP�ܵı��س�����Ԥ��
    Overum=TRUE;
  OverBits=-R;//R���෴�� �����ı�����(����ʾ�г��� ����ʾû�г���)

  /*initialize the lower bound and the upper bound for the target bits of each frame, HRD consideration*/
  LowerBound=(long)(R+bit_rate/frame_rate);//�½�
  UpperBound1=(long)(R+InitialDelayOffset);//�Ͻ�   InitialDelayOffset=BufferSize*0.8; BufferSize=bit_rate*2.56;

 /*compute the total number of bits for the current GOP*/ 
  //(1 + np + nb) I,P,B֡���� ��GOP��֡����
  AllocatedBits = (int) floor((1 + np + nb) * bit_rate / frame_rate + 0.5);//�����bits
  R +=AllocatedBits;//ǰһ��GOPʣ���bits���������GOPԤ���bits
  Np  = np;
  Nb  = nb;

  OverDuantQp=(int)(8*OverBits/AllocatedBits+0.5);//??
  GOPOverdue=FALSE;
  
/*field coding*/
  img->IFLAG=1;

/*Compute InitialQp for each GOP*/
  TotalPFrame=np;//p֡����
  img->NumberofGOP++;//GOP��ʼ�ۼ�
  if(img->NumberofGOP==1)//����ǵ�һ��GOP
  {
    MyInitialQp=input->SeinitialQP;//��һ��GOP��IDRͼ��QPȷ��
    PreviousQp2=MyInitialQp-1; //recent change -0;
    QPLastGOP=MyInitialQp;//Ϊ��������GOP��IDRͼ���QP
  
  }
  else//������һ��GOP
  {
/*adaptive field/frame coding*/
    if((input->PicInterlace==ADAPTIVE_CODING)\
      ||(input->MbInterlace))
    {
      if (img->FieldFrame == 1)//ͼ����֡?
      {
        img->TotalQpforPPicture += FrameQPBuffer;//��ǰһ��P֡��QP��ӵ��������
        QPLastPFrame = FrameQPBuffer;//Ϊ��������GOP��IDRͼ���QP
      }
      else//ͼ���ǳ�?
      {
        img->TotalQpforPPicture += FieldQPBuffer;
        QPLastPFrame = FieldQPBuffer;
      }
      
    }
    /*compute the average QP of P frames in the previous GOP*/
    PAverageQp=(int)(1.0*img->TotalQpforPPicture/img->NumberofPPicture+0.5);//ǰһ��GOP������P֡��ƽ��QP 

    GOPDquant=(int)(0.5+1.0*(np+nb+1)/15);//ƫ�� p166
    if(GOPDquant>2)
        GOPDquant=2;//�޷�

    PAverageQp-=GOPDquant;
	//I֮֡ǰ�϶������һ��P֡
    if (PAverageQp > (QPLastPFrame - 2))//QPLastPFrame��ǰһ��GOP���һ��P֡ͼ���QP
      PAverageQp--;
    PAverageQp = MAX(QPLastGOP-2,  PAverageQp);//QPLastGOP����һ��GOP��IDRͼ���QP
    PAverageQp = MIN(QPLastGOP+2, PAverageQp);
    PAverageQp = MIN(RC_MAX_QUANT, PAverageQp);//RC_MAX_QUANT��51
    PAverageQp = MAX(RC_MIN_QUANT, PAverageQp);//�����0
  

    MyInitialQp=PAverageQp;//�ǵ�һ��GOP��IDR֡��ʼ����QP 
    QPLastGOP = MyInitialQp;//�������� Ϊ�����һ��GOP��IDR֡QP��׼��
    Pm_Qp=PAverageQp;
    PAveFrameQP=PAverageQp;
    PreviousQp1=PreviousQp2;
    PreviousQp2=MyInitialQp-1;  
  }

  img->TotalQpforPPicture=0;//�����Ѿ�ʹ�ù������� ����ط����� ��ǰ֡����������˺��ٸ�ֵ
  img->NumberofPPicture=0;
  NumberofBFrames=0; 
}

void rc_init_pict(int fieldpic,int topfield,int targetcomputation)
{
  int i;

/*compute the total number of basic units in a frame*/
  if(input->MbInterlace)
    TotalNumberofBasicUnit=img->Frame_Total_Number_MB/img->BasicUnit;//������Ԫ�ĸ��� ������Ԫ�����ɸ���鹹��
  img->NumberofCodedMacroBlocks=0;

/*Normally, the bandwith for the VBR case is estimated by 
a congestion(ӵ��) control algorithm. A bandwidth curve can be predefined if we only want to 
test the proposed(����) algorithm*/
  if(input->channel_type==1)//ʱ���ŵ�
  {
    if(img->NumberofCodedPFrame==58)//���Ǻܶ�?? ����ͻȻ���� ���ڳ���������Կɱ����ʿ���??
      bit_rate *=1.5;
    else if(img->NumberofCodedPFrame==59)
      PreviousBit_Rate=bit_rate;
  }

/*predefine a target buffer level for each frame*/
  if((fieldpic||topfield)&&targetcomputation)
  {
    switch (img->type)
    {
      case P_SLICE:  //PƬ �����ʿ��������㷨�õ����ĵط�
/*Since the available bandwidth may vary at any time, the total number of 
bits is updated picture by picture ÿ֡ͼ�񶼸���R */
        if(PreviousBit_Rate!=bit_rate)//������Ǻ㶨����
          R +=(int) floor((bit_rate-PreviousBit_Rate)*(Np+Nb)/frame_rate+0.5);//
              
/* predefine the  target buffer level for each picture.
frame layer rate control*/
        if(img->BasicUnit==img->Frame_Total_Number_MB)//һ��������Ԫ�����к�� �൱��ͼ�����ʿ���
        {
          if(img->NumberofPPicture==1)//��ǰGOP��һ��P֡
          {
            TargetBufferLevel=CurrentBufferFullness;//Si(2)=Vi(2) ��������ӯ�̶ȸ�ֵ������������
            DeltaP=(CurrentBufferFullness-GOPTargetBufferLevel)/(TotalPFrame-1);//����û�м�GOPTargetBufferLevel
            TargetBufferLevel -=DeltaP;
          }
          else if(img->NumberofPPicture>1)//��ǰGOP�ǵ�һ��P֡
            TargetBufferLevel -=DeltaP;
        }
/*basic unit layer rate control*/
        else//һ��������Ԫ�Ǽ�����һ����� ������Ԫ�����ʿ���
        {
          if(img->NumberofCodedPFrame>0)
          {
            /*adaptive frame/filed coding*/
            if(((input->PicInterlace==ADAPTIVE_CODING)||(input->MbInterlace))\
              &&(img->FieldControl==1))
            {
              for(i=0;i<TotalNumberofBasicUnit;i++)
                FCBUPFMAD[i]=FCBUCFMAD[i];//��һ��������Ԫͬλ�õ�MADS��Ԥ�⵱ǰ��MADS
            }
            else
            {
              for(i=0;i<TotalNumberofBasicUnit;i++)
                BUPFMAD[i]=BUCFMAD[i];
            }     
          }
			//��ʵGOP���жϿ���ȥ�� ������һ��
          if(img->NumberofGOP==1)//��һ��GOP
          {
            if(img->NumberofPPicture==1)//��һ��GOP��һ��P֡
            {
              TargetBufferLevel=CurrentBufferFullness;//��ͼ������ʿ���һ�� Si(2)=Vi(2) 
              DeltaP=(CurrentBufferFullness-GOPTargetBufferLevel)/(TotalPFrame-1);
              TargetBufferLevel -=DeltaP;
            }
            else if(img->NumberofPPicture>1)//��һ��GOP�����P֡
              TargetBufferLevel -=DeltaP;
          }
          else if(img->NumberofGOP>1)//�ǵ�һ��GOP
          {
            if(img->NumberofPPicture==0)//����GOP�ĵ�һ��P֡
            {
              TargetBufferLevel=CurrentBufferFullness;
              DeltaP=(CurrentBufferFullness-GOPTargetBufferLevel)/TotalPFrame;
              TargetBufferLevel -=DeltaP;
            }
            else if(img->NumberofPPicture>0)//����GOP�ķǵ�һ��P֡
              TargetBufferLevel -=DeltaP;
          }
        }

        if(img->NumberofCodedPFrame==1)//ÿһ��GOP�ĵ�һ��P֡
          AWp=Wp;//Wp��P֡ͼ��ĸ��Ӷ� AWp��P֡ͼ���ƽ�����Ӷ�
        if((img->NumberofCodedPFrame<8)&&(img->NumberofCodedPFrame>1))//ÿһ��GOP�ĵڶ���P֡�����߸�P֡
            AWp=Wp*(img->NumberofCodedPFrame-1)/img->NumberofCodedPFrame+\
              AWp/img->NumberofCodedPFrame;
          else if(img->NumberofCodedPFrame>1)//����ж��е��??
            AWp=Wp/8+7*AWp/8;//�����й�ʽ
          
        //compute the average complexity of B frames
        if(input->successive_Bframe>0)//�������B֡ IPBP
        {
          //compute the target buffer level  
          //TargetBufferLevel��B֡����L(input->successive_Bframe)�й�
          TargetBufferLevel +=(AWp*(input->successive_Bframe+1)*bit_rate\
            /(frame_rate*(AWp+AWb*input->successive_Bframe))-bit_rate/frame_rate);//���ʱ��TargetBufferLevel�������������
        }
        
        break;
		//��ͼ��� ��Ϊ�ǲο�֡ PBB..BPͨ�����ڵ�P֡��QP�����м�B֡��QP 
		//�ڻ������뵥Ԫ�� ����ͼ��Ԫʹ����ͬ��QP
         case B_SLICE: 	
         /* update the total number of bits if the bandwidth is changed*/
           if(PreviousBit_Rate!=bit_rate)//�ɱ����ʿ���
             R +=(int) floor((bit_rate-PreviousBit_Rate)*(Np+Nb)/frame_rate+0.5);//ͼ�������bits
            if((img->NumberofCodedPFrame==1)&&(img->NumberofCodedBFrame==1))//��ǰGOP�����һ��B֡
          {
            AWp=Wp;//Wp��P֡ͼ��ĸ��Ӷ� AWp��P֡ͼ���ƽ�����Ӷ�
            AWb=Wb;//Wp��B֡ͼ��ĸ��Ӷ� AWp��B֡ͼ���ƽ�����Ӷ�
          }
          else if(img->NumberofCodedBFrame>1)//��ǰGOP��������B֡
          {
            //compute the average weight
            if(img->NumberofCodedBFrame<8)
              AWb=Wb*(img->NumberofCodedBFrame-1)/img->NumberofCodedBFrame+\
                AWb/img->NumberofCodedBFrame;
            else
              AWb=Wb/8+7*AWb/8;
          }

            break;
    }
     /*Compute the target bit for each frame*/
    if(img->type==P_SLICE)//����P֡ͼ���Ŀ��bits
    {
      /*frame layer rate control*/
      if(img->BasicUnit==img->Frame_Total_Number_MB)//һ��������Ԫ���к�� �൱��ͼ���������
      {
        if(img->NumberofCodedPFrame>0)
        {
        //�ӵ�ǰGOP����bit�ĽǶ�������
          T = (long) floor(Wp*R/(Np*Wp+Nb*Wb) + 0.5);//����Ȩ�صļ���ƽ��һ֡�ο�ͼ����Ҫ��bits
          //��Ŀ������ ֡�� ������ռ����� Ŀ�껺�漶��
          T1 = (long) floor(bit_rate/frame_rate-GAMMAP*(CurrentBufferFullness-TargetBufferLevel)+0.5);
          T1=MAX(0,T1);
          T = (long)(floor(BETAP*T+(1.0-BETAP)*T1+0.5));
        }
       }
      /*basic unit layer rate control*/
      else//����������ʿ���
      {
        if((img->NumberofGOP==1)&&(img->NumberofCodedPFrame>0))//��һ��GOP
        {
          T = (int) floor(Wp*R/(Np*Wp+Nb*Wb) + 0.5);
          T1 = (int) floor(bit_rate/frame_rate-GAMMAP*(CurrentBufferFullness-TargetBufferLevel)+0.5);
          T1=MAX(0,T1);
          T = (int)(floor(BETAP*T+(1.0-BETAP)*T1+0.5));
        }
        else if(img->NumberofGOP>1)//�����GOP
        {
          T = (long) floor(Wp*R/(Np*Wp+Nb*Wb) + 0.5);
          T1 = (long) floor(bit_rate/frame_rate-GAMMAP*(CurrentBufferFullness-TargetBufferLevel)+0.5);
          T1 = MAX(0,T1);
          T = (long)(floor(BETAP*T+(1.0-BETAP)*T1+0.5));
        }
      }

      /*reserve some bits for smoothing*/

      T=(long)((1.0-0.0*input->successive_Bframe)*T);//0��?? ��仰�������?????
      /*HRD consideration*/
      T = MAX(T, (long) LowerBound);//�ɱ��ŵ�����ʹ���
        T = MIN(T, (long) UpperBound2);//�ɱ��ŵ�����ߴ���

      if((topfield)||(fieldpic&&((input->PicInterlace==ADAPTIVE_CODING)\
        ||(input->MbInterlace))))
        T_field=T;
    }
  }

  if(fieldpic||topfield)//����
  {
    /*frame layer rate control*/
    img->NumberofHeaderBits=0;
    img->NumberofTextureBits=0;

    /*basic unit layer rate control*/
    if(img->BasicUnit < img->Frame_Total_Number_MB)//һ��������Ԫ�м������ ������Ԫ��������
    {
      TotalFrameQP=0;
      img->NumberofBasicUnitHeaderBits=0;
      img->NumberofBasicUnitTextureBits=0;
      img->TotalMADBasicUnit=0;
      if(img->FieldControl==0)
        NumberofBasicUnit=TotalNumberofBasicUnit;
      else
        NumberofBasicUnit=TotalNumberofBasicUnit/2;
    }
  }
    
  if((img->type==P_SLICE)&&(img->BasicUnit<img->Frame_Total_Number_MB)\
    &&(img->FieldControl==1)) //P֡ ��ģʽ ������Ԫ��������
  {
  /*top filed at basic unit layer rate control*/
    if(topfield)//���� 
    {
      bits_topfield=0;
      T=(long)(T_field*0.6);//����ռ����֡������bits��0.6
    }
  /*bottom filed at basic unit layer rate control*/
    else//�׳�
    {
      T=T_field-bits_topfield;//�׳�ռ����֡������bits��0.4
      img->NumberofBasicUnitHeaderBits=0;
      img->NumberofBasicUnitTextureBits=0;
      img->TotalMADBasicUnit=0;
      NumberofBasicUnit=TotalNumberofBasicUnit/2;
    }
  }
}

//calculate MAD for the current macroblock 
double calc_MAD()
{
  int k,l;
    int s = 0;
  double MAD;

  for (k = 0; k < 16; k++)
    for (l = 0; l < 16; l++)
      s+= abs(diffy[k][l]);//�в��������ÿһ��Ԫ�ؾ���ֵ���
  
  MAD=s*1.0/256;//��Ϊ��ƽ��������� ����Ҫ����16*16
  return MAD;
}

// update one picture after frame/field encoding
void rc_update_pict(int nbits)// nbits��ǰһͼ�������ʵ�ʱ�����
{
//����ط�һ��ͼ������bits����������
  R-= nbits; /* remaining # of bits in GOP ��ǰGOP��ʣ��bitsҪ��ȥ��һ֡ʵ�ʵĲ�����bits(�������ʿ��Ƽ����QP���������ı�����)
  */
  //��ǰ�������ĳ�ӯ�̶� ����©��ԭ��             		nbits - bit_rate/frame_rate��ʾ�����ӵ�bits
  CurrentBufferFullness += nbits - bit_rate/frame_rate;
	
  /*update the lower bound and the upper bound for the target bits of each frame, HRD consideration*/
  LowerBound  +=(long)(bit_rate/frame_rate-nbits);//(bit_rate/frame_rate-nbits)��ʾ�������Bits
  UpperBound1 +=(long)(bit_rate/frame_rate-nbits);
  UpperBound2 = (long)(OMEGA*UpperBound1);
  
  return;
}

// update after frame encoding ������պ���ѱ���B P֡�ļ������ۼӣ���ȡ����ͷ����Ϣ������ͼ���Ӷ�
void rc_update_pict_frame(int nbits)
{

/*update the
complexity weight of I, P, B frame*/
  int Avem_Qc;
  int X;
    
/*frame layer rate control*/
  if(img->BasicUnit==img->Frame_Total_Number_MB)//ͼ������ʿ���
    X = (int) floor(nbits*m_Qc+ 0.5);//���㸴�Ӷ�??
/*basic unit layer rate control*/
  else//�������뵥Ԫ�����ʿ���
  {
    if(img->type==P_SLICE)//P֡
    {
      if(((img->IFLAG==0)&&(img->FieldControl==1))\
        ||(img->FieldControl==0))
      {
        Avem_Qc=TotalFrameQP/TotalNumberofBasicUnit;//һ��Pͼ�������ƽ��QP����һ��Pͼ������ÿһ�����뵥Ԫ��ƽ������õ�
        X=(int)floor(nbits*Avem_Qc+0.5);
      }
    }
    else if(img->type==B_SLICE)//B֡
      X = (int) floor(nbits*m_Qc+ 0.5);
  }


  switch (img->type)
  {
     //P֡
    case P_SLICE:
 /*filed coding*/
      if(((img->IFLAG==0)&&(img->FieldControl==1))\
        ||(img->FieldControl==0))
      {
        Xp = X;
        Np--;//NP���� ˵��NP�Ǳ�ʾ��ǰGOP��ʣ���P֡��
        Wp=Xp;//����P֡ͼ���Ӷ� ��ͼ��ʵ�ʲ����ı�������ƽ��QP�й�
        Pm_Hp=img->NumberofHeaderBits;//ͷ��bit
        img->NumberofCodedPFrame++;
        img->NumberofPPicture++;
      }
      else if((img->IFLAG!=0)&&(img->FieldControl==1))
        img->IFLAG=0;
        break;
        case B_SLICE://B֡
        Xb = X;
        Nb--;
        Wb=Xb/THETA;// ����B֡ͼ���Ӷ� ��ͼ��ʵ�ʲ����ı�������ƽ��QP�й�
    
        img->NumberofCodedBFrame++;
        NumberofBFrames++;

        break;
    }
}

// coded bits for top field
void setbitscount(int nbits)
{
  bits_topfield = nbits;
}

//compute a  quantization parameter for each frame
//����ÿһ֡����������	QP      	�õ����ʿ��Ƶ����ս�� 
int updateQuantizationParameter(int topfield)
{
  double dtmp;
  int m_Bits;
  int BFrameNumber;
  int StepSize;
  int PAverageQP;
  int SumofBasicUnit;
  int i;
  
/*frame layer rate control*/
  if(img->BasicUnit==img->Frame_Total_Number_MB)//����ͼ�������ʿ���
  {
/*fixed quantization parameter is used to coded I frame, the first P frame and the first B frame
  the quantization parameter is adjusted according the available channel bandwidth and 
  the type of vide*/  
/*top field*/
    if((topfield)||(img->FieldControl==0))//����
    {
      if(img->type==I_SLICE)//ͼ����I֡
      {
        m_Qc=MyInitialQp;//GOP�ĵ�һ֡IDRͼ��QP 
        return m_Qc;
      }
      else if(img->type==B_SLICE)//BƬ
      {
        if(input->successive_Bframe==1)//һ��GOPֻ��һ��B֡ PBP
        {
        //PBB..BBBP BFrameNumber�ǵ�ǰ����֡�ṹ�����B֡˳�� 12...56
          BFrameNumber=(NumberofBFrames+1)%input->successive_Bframe;
          if(BFrameNumber==0)
            BFrameNumber=input->successive_Bframe;
          /*adaptive field/frame coding*/
          else if(BFrameNumber==1)//��ǰһС��(����P֡��)����ĵ�һ��B֡
          {
            if((input->PicInterlace==ADAPTIVE_CODING)\
              ||(input->MbInterlace))//��ȡ��֯
            {
              if(img->FieldControl==0)
              {                   
                /*previous choice is frame coding*/
                if(img->FieldFrame==1)//֡
                {
                //PreviousQp2 B FrameQPBuffer
                //PreviousQp1 B PreviousQp2 B FrameQPBuffer
                //			    PreviousQp1 B PreviousQp2
                  PreviousQp1=PreviousQp2;//PBP������ߵ�P֡QP
                  PreviousQp2=FrameQPBuffer;//PBP�����ұߵ�P֡QP
                }           
                /*previous choice is field coding*/
                else//��
                {
                  PreviousQp1=PreviousQp2;
                  PreviousQp2=FieldQPBuffer;
                }
              }
            }
          }
          if(PreviousQp1==PreviousQp2)//PBP�Ľṹ ���ǰ��P֡����������һ�� B֡������������2
            m_Qc=PreviousQp1+2;
          else////PBP�Ľṹ ���ǰ��P֡������������һ�� Ϊ�˱�֤ƽ�� ����������
            m_Qc=(PreviousQp1+PreviousQp2)/2+1;
          m_Qc = MIN(m_Qc, RC_MAX_QUANT); // clipping
          m_Qc = MAX(RC_MIN_QUANT, m_Qc);//clipping
        }
        else//һ��GOPֻ�ж��B֡ PBB..BP
        {
         //PBB..BBBP BFrameNumber�ǵ�ǰ����֡�ṹ�����B֡˳�� 12...56
          BFrameNumber=(NumberofBFrames+1)%input->successive_Bframe;
          if(BFrameNumber==0)
            BFrameNumber=input->successive_Bframe;
          /*adaptive field/frame coding*/
          else if(BFrameNumber==1)//��ǰһС��(����P֡��)����ĵ�һ��B֡
          {
            if((input->PicInterlace==ADAPTIVE_CODING)\
              ||(input->MbInterlace))
            {
              if(img->FieldControl==0)
              {
                /*previous choice is frame coding*/
                if(img->FieldFrame==1)
                {
                  PreviousQp1=PreviousQp2;//PBB...BP������ߵ�P֡QP
                  PreviousQp2=FrameQPBuffer;//PBB...BP�����ұߵ�P֡QP
                }
                /*previous choice is field coding*/
                else
                {
                  PreviousQp1=PreviousQp2;
                  PreviousQp2=FieldQPBuffer;
                }
              }
            }
          }

          if((PreviousQp2-PreviousQp1)<=(-2*input->successive_Bframe-3))
            StepSize=-3;
          else  if((PreviousQp2-PreviousQp1)==(-2*input->successive_Bframe-2))
            StepSize=-2;
          else if((PreviousQp2-PreviousQp1)==(-2*input->successive_Bframe-1))
            StepSize=-1;
          else if((PreviousQp2-PreviousQp1)==(-2*input->successive_Bframe))
            StepSize=0;
          else if((PreviousQp2-PreviousQp1)==(-2*input->successive_Bframe+1))
            StepSize=1;
          else
            StepSize=2;
          
          m_Qc=PreviousQp1+StepSize;
          m_Qc +=MIN(2*(BFrameNumber-1),MAX(-2*(BFrameNumber-1), \
            (BFrameNumber-1)*(PreviousQp2-PreviousQp1)/(input->successive_Bframe-1)));//�����(BFrameNumber-1)
          m_Qc = MIN(m_Qc, RC_MAX_QUANT); // clipping �޷�
          m_Qc = MAX(RC_MIN_QUANT, m_Qc);//clipping
        }
        return m_Qc;
      }
      else if((img->type==P_SLICE)&&(img->NumberofPPicture==0))//��һ��IDR֡(I��P)
      {
        m_Qc=MyInitialQp;//����ֱ���õ�I֡��QP���õ�
        
        if(img->FieldControl==0)
        {
          if(active_sps->frame_mbs_only_flag)
          {
            img->TotalQpforPPicture +=m_Qc;//���浱ǰ��QP �ۼӵ�����P֡QP�͵ı�������
            PreviousQp1=PreviousQp2;
            PreviousQp2=m_Qc;
            Pm_Qp=m_Qc;
          }
          /*adaptive field/frame coding*/
          else
            FrameQPBuffer=m_Qc;
        }
        
        return m_Qc;  
      }
      else//�����P֡
      {
      /*
		��һ��:ȷ��ÿһ���ο�ͼ��(P)��Ŀ��Ŀ��bits
		�ڶ���:ȷ���ο�ͼ���QP ����RDO�Ż�
	  */
        /*adaptive field/frame coding*/
        if(((input->PicInterlace==ADAPTIVE_CODING)\
          ||(input->MbInterlace))\
          &&(img->FieldControl==0))
        {
          /*previous choice is frame coding*/
          if(img->FieldFrame==1)
          {
            img->TotalQpforPPicture +=FrameQPBuffer;
            Pm_Qp=FrameQPBuffer;
          }
          /*previous choice is field coding*/
          else
          {
            img->TotalQpforPPicture +=FieldQPBuffer;
            Pm_Qp=FieldQPBuffer;//ǰһ֡P֡��QP
          }
        }
		//Pm_X1=bit_rate*1.0;//RDģ�Ͳ���
   		//Pm_X2=0.0;
        m_X1=Pm_X1;//RDģ�͵���������
        m_X2=Pm_X2;
        m_Hp=PPreHeader;
        m_Qp=Pm_Qp;//ǰһ֡P֡��QP
        DuantQp=PDuantQp;//�����������ı仯�� 2
        MADPictureC1=PMADPictureC1;//PMADPictureC1=1 ����Ԥ�����������
        MADPictureC2=PMADPictureC2;//PMADPictureC1=0
        PreviousPictureMAD=PPictureMAD[0];
        
        /* predict the MAD of current picture*/
        CurrentFrameMAD=MADPictureC1*PreviousPictureMAD+MADPictureC2;//����Ԥ�⵱ǰ֡��MAD
        
        /*compute the number of bits for the texture*/      
        
        if(T<0)//��TС��0 ֱ��ʹ��ǰһ��P֡��QP��Ԥ�� Ȼ���2 �õ���ǰ��QPֵ
        {
          m_Qc=m_Qp+DuantQp;//ǰһ֡P֡��QP��2 DuantQp=2
          m_Qc = MIN(m_Qc, RC_MAX_QUANT); // clipping
        }
        else//��T����0 ִ��RD�Ż�
        {
          m_Bits =T-m_Hp;//��ȥͷ��bits ??
          m_Bits = MAX(m_Bits, (int)(bit_rate/(MINVALUE*frame_rate)));//MINVALUE=4 ȷ��һ�����ֵ�Ͻ�
          //���m_Bits(Qstep)=X1*D/Qstep+x2*D/(Qstep*Qstep)һԪ���η��̽�
          dtmp = CurrentFrameMAD * m_X1 * CurrentFrameMAD * m_X1 \
            + 4 * m_X2 * CurrentFrameMAD * m_Bits;//�б�ʽ
          if ((m_X2 == 0.0) || (dtmp < 0) || ((sqrt (dtmp) - m_X1 * CurrentFrameMAD) <= 0.0)) // fall back 1st order mode
            m_Qstep = (float) (m_X1 * CurrentFrameMAD / (double) m_Bits);
          else // 2nd order mode
            m_Qstep = (float) ((2 * m_X2 * CurrentFrameMAD) / (sqrt (dtmp) - m_X1 * CurrentFrameMAD));
          
          m_Qc=Qstep2QP(m_Qstep);//0.625��224��Qstepӳ�䵽QP 0��51
          //m_Qp��ǰһP֡QP��Ԥ��
          m_Qc = MIN(m_Qp+DuantQp,  m_Qc);  // control variation
          m_Qc = MIN(m_Qc, RC_MAX_QUANT); // clipping
          m_Qc = MAX(m_Qp-DuantQp, m_Qc); // control variation
          m_Qc = MAX(RC_MIN_QUANT, m_Qc);//	QPƽ�����޷�
        }
        
        if(img->FieldControl==0)
        {
          /*frame coding*/
          if(active_sps->frame_mbs_only_flag)
          {
            img->TotalQpforPPicture +=m_Qc;//��¼�ۼ����ڵ�QP �����һ��GOP��IDR֡��QP
            PreviousQp1=PreviousQp2;//��¼����P֡��QP Ϊ�˼���B֡QP
            PreviousQp2=m_Qc;
            Pm_Qp=m_Qc;//��ǰP֡��QP����һ֡P֡Ԥ��
          }
          /*adaptive field/frame coding*/
          else
            FrameQPBuffer=m_Qc;
        }
        
        return m_Qc;
      }
   }
   /*bottom field*/
   else//�׳�
   {
     if((img->type==P_SLICE)&&(img->IFLAG==0))
     {
       /*field coding*/
       if(input->PicInterlace==FIELD_CODING)
       {
         img->TotalQpforPPicture +=m_Qc;//��ʲô�ط����??
         PreviousQp1=PreviousQp2+1; 
         PreviousQp2=m_Qc;//+0 Recent change 13/1/2003
         Pm_Qp=m_Qc;
       }
       /*adaptive field/frame coding*/
       else
         FieldQPBuffer=m_Qc;     
     }
     return m_Qc;
   }
  }
  /*basic unit layer rate control*/
  else //�������뵥Ԫ������ʿ���        		I֡��P֡����ǰ��ͼ������ʿ���һ��
  {
    /*top filed of I frame*/
    if(img->type==I_SLICE)//I֡�����б��뵥Ԫ�ĺ��QP��ͬ
    {
      m_Qc=MyInitialQp;
      return m_Qc;
    }
    /*bottom field of I frame*/
    else if((img->type==P_SLICE)&&(img->IFLAG==1)&&(img->FieldControl==1))
    {
      m_Qc=MyInitialQp;
      return m_Qc;
    }
    else if(img->type==B_SLICE)//B֡
    {
      /*top filed of B frame*/
      if((topfield)||(img->FieldControl==0))//B֡����
      {
        if(input->successive_Bframe==1)//PBP ֻ��һ��B
        {
          BFrameNumber=(NumberofBFrames+1)%input->successive_Bframe;
        if(BFrameNumber==0)
          BFrameNumber=input->successive_Bframe;
        /*adaptive field/frame coding*/
        else if(BFrameNumber==1)
        {
          if((input->PicInterlace==ADAPTIVE_CODING)\
            ||(input->MbInterlace))
          {
            if(img->FieldControl==0)
            {             
        /*previous choice is frame coding*/
              if(img->FieldFrame==1)
              {
                PreviousQp1=PreviousQp2;
                PreviousQp2=FrameQPBuffer;
              }
        /*previous choice is field coding*/
              else
              {
                PreviousQp1=PreviousQp2;
                PreviousQp2=FieldQPBuffer;
              }
            }
          }
        }
          if(PreviousQp1==PreviousQp2)
            m_Qc=PreviousQp1+2;
          else
            m_Qc=(PreviousQp1+PreviousQp2)/2+1;
          m_Qc = MIN(m_Qc, RC_MAX_QUANT); // clipping
          m_Qc = MAX(RC_MIN_QUANT, m_Qc);//clipping
        }
        else//PBB...BP��ʽ
        {
        /*
			����PBBBBP    input->successive_Bframe=4
			NumberofBFrames=0  BFrameNumber=1
			NumberofBFrames=1  BFrameNumber=2
			NumberofBFrames=2  BFrameNumber=3
			NumberofBFrames=3  BFrameNumber=0
			BFrameNumber��һ��֡�ṹ�ڲ���B֡˳��
		*/
          BFrameNumber=(NumberofBFrames+1)%input->successive_Bframe;
        if(BFrameNumber==0)
          BFrameNumber=input->successive_Bframe;
        /*adaptive field/frame coding*/
        else if(BFrameNumber==1)//PBBBP�ĵ�һ��B
        {
          if((input->PicInterlace==ADAPTIVE_CODING)\
            ||(input->MbInterlace))
          {
            if(img->FieldControl==0)
            {
          /*previous choice is frame coding*/
              if(img->FieldFrame==1)
              {
              /*
				PreviousQp2  FrameQPBuffer
				PreviousQp1  PreviousQp2  FrameQPBuffer
							 PreviousQp1  PreviousQp2  FrameQPBuffer
			  */
                PreviousQp1=PreviousQp2;
                PreviousQp2=FrameQPBuffer;
              }
          /*previous choice is field coding*/
              else
              {
                PreviousQp1=PreviousQp2;
                PreviousQp2=FieldQPBuffer;
              }
            } 
          }
        }

        if((PreviousQp2-PreviousQp1)<=(-2*input->successive_Bframe-3))
          StepSize=-3;
        else  if((PreviousQp2-PreviousQp1)==(-2*input->successive_Bframe-2))
          StepSize=-2;
        else if((PreviousQp2-PreviousQp1)==(-2*input->successive_Bframe-1))
          StepSize=-1;
        else if((PreviousQp2-PreviousQp1)==(-2*input->successive_Bframe))
          StepSize=0;//0
        else if((PreviousQp2-PreviousQp1)==(-2*input->successive_Bframe+1))
          StepSize=1;//1
        else
          StepSize=2;//2
        m_Qc=PreviousQp1+StepSize;
        m_Qc +=MIN(2*(BFrameNumber-1),MAX(-2*(BFrameNumber-1), \
          (BFrameNumber-1)*(PreviousQp2-PreviousQp1)/(input->successive_Bframe-1)));
        m_Qc = MIN(m_Qc, RC_MAX_QUANT); // clipping
        m_Qc = MAX(RC_MIN_QUANT, m_Qc);//clipping
        }
        return m_Qc;
      }
      /*bottom field of B frame*/
      else
        return m_Qc;
    }
    else if(img->type==P_SLICE)//P֡�ڻ������뵥Ԫ������ʿ���
    {
      if((img->NumberofGOP==1)&&(img->NumberofPPicture==0))//��һ��GOP�ĵ�һ��P֡
      {
        if((img->FieldControl==0)||((img->FieldControl==1)\
          &&(img->IFLAG==0)))
        {
        /*top field of the first P frame*/
          m_Qc=MyInitialQp;//��I֡һ�� ʹ��fix�ķ���
          img->NumberofBasicUnitHeaderBits=0;
          img->NumberofBasicUnitTextureBits=0;
          NumberofBasicUnit--;//������Ԫ�������
        /*bottom field of the first P frame*/
          if((!topfield)&&(NumberofBasicUnit==0))//�׳��ĵ�һ��������Ԫ
          {
            /*frame coding or field coding*/
            if((active_sps->frame_mbs_only_flag)||(input->PicInterlace==FIELD_CODING))//֡���߳�
            {
              img->TotalQpforPPicture +=m_Qc;
              PreviousQp1=PreviousQp2;
              PreviousQp2=m_Qc;
              PAveFrameQP=m_Qc;//���浱ǰ��QP ??
              PAveHeaderBits3=PAveHeaderBits2;//ͷ����Ϣbits??
            }
            /*adaptive frame/field coding*/
            else if((input->PicInterlace==ADAPTIVE_CODING)\
              ||(input->MbInterlace))//��֯
            {
              if(img->FieldControl==0)//����
              {
                FrameQPBuffer=m_Qc;
                FrameAveHeaderBits=PAveHeaderBits2;
              }
              else//�׳�
              {
                FieldQPBuffer=m_Qc;
                FieldAveHeaderBits=PAveHeaderBits2;
              }
            }
          }
          Pm_Qp=m_Qc;//������Ԥ��
          TotalFrameQP +=m_Qc;//��ǰ֡�ڵ�QP���ۼ�
          return m_Qc;
        }
      }
      else// �ǵ�һ֡��P֡ ʹ��RD�Ż��㷨
      {
        m_X1=Pm_X1;//RD�Ż�����������
        m_X2=Pm_X2;
        m_Hp=PPreHeader;//ͷ����Ϣ
        m_Qp=Pm_Qp;//Ԥ��QP
        DuantQp=PDuantQp;//QP��ƫ����2
        MADPictureC1=PMADPictureC1;//MAD����Ԥ��Ĳ���
        MADPictureC2=PMADPictureC2;

        if(img->FieldControl==0)//�׳�
          SumofBasicUnit=TotalNumberofBasicUnit;
        else//����
          SumofBasicUnit=TotalNumberofBasicUnit/2;
		/*
		TotalNumberofBasicUnit:���ܵĻ������뵥Ԫ��
		NumberofBasicUnit:˵�����ֵ����ǵ�һ�����뵥Ԫ Ϊ0�����һ�����뵥Ԫ
		*/
        /*the average QP of the previous frame is used to coded the first basic unit of the current frame or field*/
        if(NumberofBasicUnit==SumofBasicUnit)//�����뵽��һ�����뵥Ԫ ����������ƽ��
        {

          /*adaptive field/frame coding*/
          if(((input->PicInterlace==ADAPTIVE_CODING)\
            ||(input->MbInterlace))\
            &&(img->FieldControl==0))//��֯�ҵ׳�
          {
            /*previous choice is frame coding*/
            if(img->FieldFrame==1)
            {
              if(img->NumberofPPicture>0)
                img->TotalQpforPPicture +=FrameQPBuffer;
              PAveFrameQP=FrameQPBuffer;
              PAveHeaderBits3=FrameAveHeaderBits;
            }       
            /*previous choice is field coding*/
            else
            {
              if(img->NumberofPPicture>0)
                img->TotalQpforPPicture +=FieldQPBuffer;
              PAveFrameQP=FieldQPBuffer;
              PAveHeaderBits3=FieldAveHeaderBits;
            }
          }

          if(T<=0)//��������bits������ 
          {
            m_Qc=PAveFrameQP+2;//����QP
            if(m_Qc>RC_MAX_QUANT)//�޷�
              m_Qc=RC_MAX_QUANT;
            if(topfield||(img->FieldControl==0))
              GOPOverdue=TRUE;//�����־λ�ڸ�ɶ??
          }
          else//���ͼ������bits����
          {
            m_Qc=PAveFrameQP; //����Ԥ������QP
          }
          TotalFrameQP +=m_Qc;
          NumberofBasicUnit--;
          Pm_Qp=PAveFrameQP;
          return m_Qc;
        }
		else//�����뵽�����һ�����뵥Ԫ
         {
          /*compute the number of remaining bits ���㵱ǰͼ��ʣ��bits*/
           TotalBasicUnitBits=img->NumberofBasicUnitHeaderBits+img->NumberofBasicUnitTextureBits;//��ǰ���뵥Ԫͷ���ͱ�������bits????
           T -=TotalBasicUnitBits;//��ǰ֡ͼ������bitsǰȥ��һ�����뵥Ԫʵ���õ�bits
           img->NumberofBasicUnitHeaderBits=0;//���
           img->NumberofBasicUnitTextureBits=0;
           if(T<0)//���Ԥ���bits����������
           {
             if(GOPOverdue==TRUE)//����QP ʹ����
               m_Qc=m_Qp+2;
             else 
               m_Qc=m_Qp+DDquant;//2 
             m_Qc = MIN(m_Qc, RC_MAX_QUANT);  // clipping  �޷�
             if(input->basicunit>=MBPerRow)//������Ԫ������ÿһ�еĺ����
               m_Qc = MIN(m_Qc, PAveFrameQP+6); 
             else
               m_Qc = MIN(m_Qc, PAveFrameQP+3);

             TotalFrameQP +=m_Qc;//�ۼӵ����֡QP����
             NumberofBasicUnit--;//�ǵݼ� ˵�����ֵ����ǵ�һ�����뵥Ԫ Ϊ0�����һ�����뵥Ԫ
             if(NumberofBasicUnit==0)//�����뵥Ԫ
             {
               if((!topfield)||(img->FieldControl==0))
               {
                 /*frame coding or field coding*/
                 if((active_sps->frame_mbs_only_flag)||(input->PicInterlace==FIELD_CODING))
                 {
                   PAverageQP=(int)(1.0*TotalFrameQP/TotalNumberofBasicUnit+0.5);//�õ�֡��ƽ��QP
                   if (img->NumberofPPicture == (input->intra_period - 2))//���Ǻܶ�???
                          QPLastPFrame = PAverageQP;

                   img->TotalQpforPPicture +=PAverageQP;
                   if(GOPOverdue==TRUE)
                   {
                     PreviousQp1=PreviousQp2+1;
                     PreviousQp2=PAverageQP;                   
                   }
                   else
                   {
                     if((img->NumberofPPicture==0)&&(img->NumberofGOP>1))
                     {
                       PreviousQp1=PreviousQp2;
                       PreviousQp2=PAverageQP;
                     }
                     else if(img->NumberofPPicture>0)
                     {
                        PreviousQp1=PreviousQp2+1;
                        PreviousQp2=PAverageQP;
                     }
                   }
                   PAveFrameQP=PAverageQP;
                   PAveHeaderBits3=PAveHeaderBits2;
                 }
                 /*adaptive field/frame coding*/
                 else if((input->PicInterlace==ADAPTIVE_CODING)\
                   ||(input->MbInterlace))
                 {
                   if(img->FieldControl==0)
                   {
                     PAverageQP=(int)(1.0*TotalFrameQP/TotalNumberofBasicUnit+0.5);
                     FrameQPBuffer=PAverageQP;
                     FrameAveHeaderBits=PAveHeaderBits2;
                   }
                   else
                   {
                     PAverageQP=(int)(1.0*TotalFrameQP/TotalNumberofBasicUnit+0.5);
                     FieldQPBuffer=PAverageQP;
                     FieldAveHeaderBits=PAveHeaderBits2;
                   }
                 }
               }
             }
             if(GOPOverdue==TRUE)
               Pm_Qp=PAveFrameQP;
             else
               Pm_Qp=m_Qc;
             return m_Qc;
           }
           else//ͼ������bits���� ʹ���Ż��㷨
           {
             /*predict the MAD of current picture*/
             if(((input->PicInterlace==ADAPTIVE_CODING)||(input->MbInterlace))\
               &&(img->FieldControl==1))
             {
             //����Ԥ�⵱ǰ��MAD ��������ط��ҵ�Ԥ��ֵ���ú�Զ???
               CurrentFrameMAD=MADPictureC1*FCBUPFMAD[TotalNumberofBasicUnit-NumberofBasicUnit]+MADPictureC2;
               TotalBUMAD=0;
			   //TotalNumberofBasicUnit=10  NumberofBasicUnit=3(��7�����뵥Ԫ) 
			   //TotalNumberofBasicUnit-NumberofBasicUnit��Ŀǰ�ڼ������뵥Ԫ
               for(i=TotalNumberofBasicUnit-1; i>=(TotalNumberofBasicUnit-NumberofBasicUnit);i--)
               {
                 CurrentBUMAD=MADPictureC1*FCBUPFMAD[i]+MADPictureC2;//ǰ���Ѿ�����ı��뵥Ԫ
                 TotalBUMAD +=CurrentBUMAD*CurrentBUMAD;//ǰ���Ѿ�����ı��뵥Ԫ��ƽ�������
               }
             }
             else
             {
               CurrentFrameMAD=MADPictureC1*BUPFMAD[TotalNumberofBasicUnit-NumberofBasicUnit]+MADPictureC2;
               TotalBUMAD=0;
               for(i=TotalNumberofBasicUnit-1; i>=(TotalNumberofBasicUnit-NumberofBasicUnit);i--)
               {
                 CurrentBUMAD=MADPictureC1*BUPFMAD[i]+MADPictureC2;
                 TotalBUMAD +=CurrentBUMAD*CurrentBUMAD;
               }
             }
                
             /*compute the total number of bits for the current basic unit*/
			 //ͨ��MAD��ռ��������bits MAD(ʧ���)Խ�� ��Ҫ�ø����bits���Ż�QP
             m_Bits =(int)(T*CurrentFrameMAD*CurrentFrameMAD/TotalBUMAD);
             /*compute the number of texture bits*/
             m_Bits -=PAveHeaderBits2;//ͷ��bits
             
             m_Bits=MAX(m_Bits,(int)(bit_rate/(MINVALUE*frame_rate*TotalNumberofBasicUnit)));//��P֡һ����ȷ��һ�����ķ��������

			 //���һԪ���η��̵��б�ʽ ʹ��RD�Ż��Ĳ���һ��
             dtmp = CurrentFrameMAD * m_X1 * CurrentFrameMAD * m_X1 \
               + 4 * m_X2 * CurrentFrameMAD * m_Bits;
             if ((m_X2 == 0.0) || (dtmp < 0) || ((sqrt (dtmp) - m_X1 * CurrentFrameMAD) <= 0.0))  // fall back 1st order mode
               m_Qstep = (float)(m_X1 * CurrentFrameMAD / (double) m_Bits);
             else // 2nd order mode
               m_Qstep = (float) ((2 * m_X2 * CurrentFrameMAD) / (sqrt (dtmp) - m_X1 * CurrentFrameMAD));

             m_Qc=Qstep2QP(m_Qstep);//��0.625��224 ӳ�䵽0��51
             m_Qc = MIN(m_Qp+DDquant,  m_Qc); // Ԥ��QP����ƫ�������Ƶ�ǰ��QP��Χcontrol variation

             if(input->basicunit>=MBPerRow)//��������
               m_Qc = MIN(PAveFrameQP+6, m_Qc);
             else
               m_Qc = MIN(PAveFrameQP+3, m_Qc);

             m_Qc = MIN(m_Qc, RC_MAX_QUANT);  // clipping
             m_Qc = MAX(m_Qp-DDquant, m_Qc);  // control variation 
             if(input->basicunit>=MBPerRow)
               m_Qc = MAX(PAveFrameQP-6, m_Qc);
             else
               m_Qc = MAX(PAveFrameQP-3, m_Qc);

             m_Qc = MAX(RC_MIN_QUANT, m_Qc);
             TotalFrameQP +=m_Qc;//��¼�����������QP �ӵ��ۼ�����
             Pm_Qp=m_Qc;//�����ڵ�QP��Ϊ��һ����Ԥ��
             NumberofBasicUnit--;//���뵥Ԫ���ݼ�
             if((NumberofBasicUnit==0)&&(img->type==P_SLICE))//P֡ �ұ��뵥Ԫ������0 �����һ�����뵥Ԫ
             {
               if((!topfield)||(img->FieldControl==0))//�׳�
               {
                 /*frame coding or field coding*/
                 if((active_sps->frame_mbs_only_flag)||(input->PicInterlace==FIELD_CODING))
                 {
                   PAverageQP=(int)(1.0*TotalFrameQP/TotalNumberofBasicUnit+0.5);//����ƽ��QP
                   if (img->NumberofPPicture == (input->intra_period - 2))
                          QPLastPFrame = PAverageQP;

                   img->TotalQpforPPicture +=PAverageQP;
                   PreviousQp1=PreviousQp2;
                   PreviousQp2=PAverageQP; 
                   PAveFrameQP=PAverageQP;
                   PAveHeaderBits3=PAveHeaderBits2;
                 }
                 else if((input->PicInterlace==ADAPTIVE_CODING)\
                   ||(input->MbInterlace))
                 {
                   if(img->FieldControl==0)
                   {
                     PAverageQP=(int)(1.0*TotalFrameQP/TotalNumberofBasicUnit+0.5);
                     FrameQPBuffer=PAverageQP;
                     FrameAveHeaderBits=PAveHeaderBits2;
                   }
                   else
                   {
                     PAverageQP=(int)(1.0*TotalFrameQP/TotalNumberofBasicUnit+0.5);
                     FieldQPBuffer=PAverageQP;
                     FieldAveHeaderBits=PAveHeaderBits2;
                   }
                 }
               }
             }
             return m_Qc;
           }
         }
      }
    } 
  }
  return m_Qc;
}

//update the parameters of quadratic R-D model
void updateRCModel ()//����ģ�͵Ĳ���
{

  int n_windowSize;
  int i;
  double error[20], std = 0.0, threshold;
  int m_Nc;
  Boolean MADModelFlag = FALSE;
   
  if(img->type==P_SLICE)//��ǰ��P֡
  {
    /*
    frame layer rate control
  */
    if(img->BasicUnit==img->Frame_Total_Number_MB)//��ǰ��ͼ�������ʿ���
    {
      CurrentFrameMAD=ComputeFrameMAD();//�õ���ǰͼ��֡��MADֵ
      m_Nc=img->NumberofCodedPFrame;//�Ѿ������P֡����
    }
    /*basic unit layer rate control*/
    else//���ڻ������뵥Ԫ�����ʿ���
    {
      /*compute the MAD of the current basic unit*/
	//��¼��ǰ�������뵥Ԫ��MAD
      if((input->MbInterlace)&&(img->FieldControl==0))//��֯�׳�
        CurrentFrameMAD=img->TotalMADBasicUnit/img->BasicUnit/2;
      else
        CurrentFrameMAD=img->TotalMADBasicUnit/img->BasicUnit;//�������뵥Ԫ��MAD��һ��ƽ��ֵ

        
      img->TotalMADBasicUnit=0;//����������ݾ��ͷ�
              
      /* compute the average number of header bits*/
      
        CodedBasicUnit=TotalNumberofBasicUnit-NumberofBasicUnit;//�Ѿ�����ı��뵥Ԫ������
        //NumberofBasicUnit=TotalNumberofBasicUnit-CodedBasicUnit ����170ҳ���뵥Ԫͷ����Ϣ����
        if(CodedBasicUnit>0)
        {
        //�����й�ʽ �Ա��뵥Ԫͷ����Ԥ��
          PAveHeaderBits1=(int)(1.0*(PAveHeaderBits1*(CodedBasicUnit-1)+\
            +img->NumberofBasicUnitHeaderBits)/CodedBasicUnit+0.5);
          if(PAveHeaderBits3==0)
            PAveHeaderBits2=PAveHeaderBits1;
          else
            PAveHeaderBits2=(int)(1.0*(PAveHeaderBits1*CodedBasicUnit+\
            +PAveHeaderBits3*NumberofBasicUnit)/TotalNumberofBasicUnit+0.5);
        }
          /*update the record of MADs for reference*/
          if(((input->PicInterlace==ADAPTIVE_CODING)||(input->MbInterlace))\
          &&(img->FieldControl==1))//��֯�Ͷ���
            FCBUCFMAD[TotalNumberofBasicUnit-1-NumberofBasicUnit]=CurrentFrameMAD;//�洢��ǰ��MAD
          else
            BUCFMAD[TotalNumberofBasicUnit-1-NumberofBasicUnit]=CurrentFrameMAD;

        if(NumberofBasicUnit!=0)//û�е����һ�����뵥Ԫ
          m_Nc=img->NumberofCodedPFrame*TotalNumberofBasicUnit+CodedBasicUnit;//����ǵ�ǰGOP�����ڵı��뵥Ԫ��˳��
        else
          m_Nc=(img->NumberofCodedPFrame-1)*TotalNumberofBasicUnit+CodedBasicUnit;
      
    }
    
    
   
    if(m_Nc>1)
      MADModelFlag=TRUE;
    PPreHeader=img->NumberofHeaderBits;//ͷ��bits
    /*
	��
	*/
    for (i = 19; i > 0; i--) {// update the history ���������ڵı���
    // Pm_rgQp[18] Pm_rgQp[17] ...... Pm_rgQp[0]
    // Pm_rgQp[19] Pm_rgQp[18] .....  Pm_rgQp[1] m_rgQp[0]=Pm_rgQp[0];
      Pm_rgQp[i] = Pm_rgQp[i - 1];//��������λ �ڳ��±�Ϊ0�ĵط� �ŵ�ǰ��QstepԤ��ֵ
      m_rgQp[i]=Pm_rgQp[i];//��Ԥ��ֵ��ֵ
	//��Pm_rgRpҲͬ������
      Pm_rgRp[i] = Pm_rgRp[i - 1];
      m_rgRp[i]=Pm_rgRp[i];
    }
    Pm_rgQp[0] = QP2Qstep(m_Qc); //*1.0/CurrentFrameMAD; ���㵱ǰQP��Qstep
    /*frame layer rate control*/
    if(img->BasicUnit==img->Frame_Total_Number_MB)//ͼ������ʿ���
    //img->NumberofTextureBits��ͼ���ʵ��ʹ�õ�bits
      Pm_rgRp[0] = img->NumberofTextureBits*1.0/CurrentFrameMAD;//�����bits����MAD ??����ɶ ����������Ż��ı������
    /*basic unit layer rate control*/
    else//������Ԫ�����ʿ���
    //img->NumberofBasicUnitTextureBits�ǻ������뵥Ԫ��ʵ��ʹ�õ�bits
      Pm_rgRp[0]=img->NumberofBasicUnitTextureBits*1.0/CurrentFrameMAD;//

    m_rgQp[0]=Pm_rgQp[0];//QP��QstepԤ��ֵ��ֵ xi
    //�����֡ͼ������bits��R(i),���߽�T(i),
    //m_rgRp[i]=R(i)/CurrentFrameMAD ����m_rgRp[i]���������ĸ�֡ͼ������bits ����һ��ӳ�� �������������
    m_rgRp[0]=Pm_rgRp[0];// yi/Di
    m_X1=Pm_X1;
    m_X2=Pm_X2;
  

/*compute the size of window*/


    n_windowSize = (CurrentFrameMAD>PreviousFrameMAD)?(int)(PreviousFrameMAD/CurrentFrameMAD*20)\
      :(int)(CurrentFrameMAD/PreviousFrameMAD*20);
    n_windowSize=MAX(n_windowSize, 1);
    n_windowSize=MIN(n_windowSize,m_Nc);//ȷ�����ڵ��Ͻ�m_Nc ��ֹ���㷶Χ����ȥ��һ���
    n_windowSize=MIN(n_windowSize,m_windowSize+1);//m_windowSize����һ��ʹ�õĴ��� ��ֹ���ڽ�Ծ����̫�� ����1�ǲ���̫����??
    n_windowSize=MIN(n_windowSize,20);

      /*update the previous window size*/
  m_windowSize=n_windowSize;
  


  for (i = 0; i < 20; i++) {//��һ�β�������ʱ��Դ���������֡ͼ�񶼴���
    m_rgRejected[i] = FALSE;
  }

  // initial RD model estimator
  RCModelEstimator (n_windowSize);//ͨ�������ڵ����ݼ�(Qstep[i],R[i]/D),RDģ�͵�m_x1,m_x2������������
 
  n_windowSize = m_windowSize;
  // remove outlier 
  
  for (i = 0; i < (int) n_windowSize; i++) {
	//��������ǿ��������ƺ�ģ�ͼ�������ô�� 
    error[i] = m_X1 / m_rgQp[i] + m_X2 / (m_rgQp[i] * m_rgQp[i]) - m_rgRp[i];
    std += error[i] * error[i]; 
  }
  //sqrt (std / n_windowSize)�Ǿ������
  threshold = (n_windowSize == 2) ? 0 : sqrt (std / n_windowSize);
  for (i = 0; i < (int) n_windowSize; i++) {
    if (fabs(error[i]) > threshold)//��Ԥ�����Ƚϴ��ͼ��֡ ���Ϊ���Ʒ���
      m_rgRejected[i] = TRUE;
  }
    // always include the last data point
  m_rgRejected[0] = FALSE;//Ĭ�ϵ�һ֡ͼ������������ ��ֹ���ݼ���

  // second RD model estimator
  RCModelEstimator (n_windowSize);//��Ԥ�����Ƚϴ��ͼ��֡�޳��� �ٴν��в�������

  if(MADModelFlag)
    updateMADModel();//��ͼ���MAD�������Իع����
  else if(img->type==P_SLICE)
    PPictureMAD[0]=CurrentFrameMAD;//
  } 
}

/*Boolean skipThisFrame ()
{
  if (m_B > (int) ((RC_SKIP_MARGIN / 100.0) * m_Bs)) {  // buffer full!
    m_skipNextFrame = TRUE;           // set the status
    m_Nr--;                   // skip one frame
    m_B -= m_Rp;                // decrease current buffer level
  } else
    m_skipNextFrame = FALSE;
  return m_skipNextFrame;
}
*/

//���ʿ���ģ�͵ĺ���
void RCModelEstimator (int n_windowSize)
{
  int n_realSize = n_windowSize;
  int i;
  double oneSampleQ;
  double a00 = 0.0, a01 = 0.0, a10 = 0.0, a11 = 0.0, b0 = 0.0, b1 = 0.0;
  double MatrixValue;
  Boolean estimateX2 = FALSE;

  for (i = 0; i < n_windowSize; i++) {// find the number of samples which are not rejected
    if (m_rgRejected[i])//�������ĳһ֡ͼ�����ݱ��ܾ�
      n_realSize--;//��ͱ�����n���� �������Ƶ����ݲ��μ��Ż�����
  }

  // default RD model estimation results

  m_X1 = m_X2 = 0.0;//���ʿ���ģ�Ͳ���������

  for (i = 0; i < n_windowSize; i++)  {
    if (!m_rgRejected[i])//
      oneSampleQ = m_rgQp[i];//m_rgQp[i]������Qstepֵ ��¼���һ��û�б����Ƶ�����
  }
  for (i = 0; i < n_windowSize; i++)  {// if all non-rejected Q are the same, take 1st order model
    if ((m_rgQp[i] != oneSampleQ) && !m_rgRejected[i])//�����ǰ�����ݲ������һ���ҵ�ǰ����û�б�����ʹ��
      estimateX2 = TRUE;//��־λ��Ϊ��
    if (!m_rgRejected[i])//�������û�б��ܾ�
      m_X1 += (m_rgQp[i] * m_rgRp[i]) / n_realSize;//Ϊɶ������???
  }

  // take 2nd order model to estimate X1 and X2 ������������????
  if ((n_realSize >= 1) && estimateX2) {
      for (i = 0; i < n_windowSize; i++) {
      if (!m_rgRejected[i]) {//����û�б�����ʹ�� 
      //���Իع��Ż�
        a00 = a00 + 1.0;
        a01 += 1.0 / m_rgQp[i];
        a10 = a01;
        a11 += 1.0 / (m_rgQp[i] * m_rgQp[i]);
        b0 += m_rgQp[i] * m_rgRp[i];
        b1 += m_rgRp[i];
      }
    }
    // solve the equation of AX = B
      MatrixValue=a00*a11-a01*a10;//A��������ʽ
      if(fabs(MatrixValue)>0.000001)//A����ʽ��Ϊ0 ����
      {
        m_X1=(b0*a11-b1*a01)/MatrixValue;
        m_X2=(b1*a00-b0*a10)/MatrixValue;
      }
      else//A������ʽΪ0 �ؽ�
      {
        m_X1=b0/a00;
        m_X2=0.0;
      }
  
  }
  if(img->type==P_SLICE)
  {
    Pm_X1=m_X1;
    Pm_X2=m_X2;
  }
}

double ComputeFrameMAD()//����ͼ��֡��MAD
{
  double TotalMAD;
  int i;
  TotalMAD=0.0;
//  CurrentFrameMAD=0.0;
  for(i=0;i<img->Frame_Total_Number_MB;i++)//�Ե�ǰͼ��֡����ĺ���������
    TotalMAD +=img->MADofMB[i];//ÿһ������MAD��������img->MADofMB[i]���� Ȼ���ۼ�
  TotalMAD /=img->Frame_Total_Number_MB;//�õ���ֵ
  return TotalMAD;
}


//update the parameters of linear prediction model
void updateMADModel ()
{
  
  int n_windowSize;
  int i;
  double error[20], std = 0.0, threshold;
  int m_Nc;
  
  if(img->NumberofCodedPFrame>0)//�Ѿ������P֡����0
  {
    
    if(img->type==P_SLICE)//P֡
    {
      /*frame layer rate control */
      if(img->BasicUnit==img->Frame_Total_Number_MB)//ͼ������ʿ���
        m_Nc=img->NumberofCodedPFrame;
      /*basic unit layer rate control*/
      else//���������ʿ���
        m_Nc=img->NumberofCodedPFrame*TotalNumberofBasicUnit+CodedBasicUnit;
      for (i = 19; i > 0; i--) {// update the history �����20������ֵ�����ڴ�С
      //PPictureMAD[18] PPictureMAD[17] ... PPictureMAD[0]
      //PPictureMAD[19] PPictureMAD[18] ... PPictureMAD[1] PPictureMAD[0] = CurrentFrameMAD 
      //PictureMAD[19]  PictureMAD[18]  ... PictureMAD[1]  PictureMAD[0]=PPictureMAD[0];
        PPictureMAD[i] = PPictureMAD[i - 1];//Ԥ���MAD�������
        PictureMAD[i]=PPictureMAD[i];//Ԥ���MAD����ʵ��MAD ����MAD����Ԥ��
        ReferenceMAD[i]= ReferenceMAD[i-1];//�ο���MAD������� ����MAD����Ԥ��
      }
      PPictureMAD[0] = CurrentFrameMAD;//��ǰ��MAD��ӵ�Ԥ��MAD�����±�Ϊ0�ĵط�
      PictureMAD[0]=PPictureMAD[0];////Ԥ���MAD����ʵ��MAD 
      if(img->BasicUnit==img->Frame_Total_Number_MB)//ͼ������ʿ���
        ReferenceMAD[0]=PictureMAD[1];//ǰһ��ͼ���MAD��Ϊ��ǰ�Ĳο�MAD
      else//���������ʿ���
      {
        if(((input->PicInterlace==ADAPTIVE_CODING)||(input->MbInterlace))\
          &&(img->FieldControl==1))
          ReferenceMAD[0]=FCBUPFMAD[TotalNumberofBasicUnit-1-NumberofBasicUnit];
        else
          ReferenceMAD[0]=BUPFMAD[TotalNumberofBasicUnit-1-NumberofBasicUnit];
      }
      MADPictureC1=PMADPictureC1;//MAD����Ԥ��Ĳ���
      MADPictureC2=PMADPictureC2;
    }
    
    
    /*compute the size of window*/
    
    n_windowSize = (CurrentFrameMAD>PreviousFrameMAD)?(int)(PreviousFrameMAD/CurrentFrameMAD*20)\
      :(int)(CurrentFrameMAD/PreviousFrameMAD*20);//ֻ�е�ǰ��ͼ��֡��MAD������ͬ��ʱ�� ��ʾͼ��ƽ�� ���ø���Ĵ���
    n_windowSize=MIN(n_windowSize,(m_Nc-1));
    n_windowSize=MAX(n_windowSize, 1);
    n_windowSize=MIN(n_windowSize,MADm_windowSize+1);//ǰһ�δ��ڵĴ�С�����Ƶ�ǰ�Ĵ��ڴ�С
    n_windowSize=MIN(20,n_windowSize);
    /*update the previous window size*/
    MADm_windowSize=n_windowSize;//���浱ǰ�Ĵ��ڴ�С
    
    for (i = 0; i < 20; i++) {//���д��������ͼ���ʼ������û�б�����ʹ�õ�
      PictureRejected[i] = FALSE;
    }
    //update the MAD for the previous frame
    if(img->type==P_SLICE)//P֡
      PreviousFrameMAD=CurrentFrameMAD;//���浱ǰͼ���MAD
    
    // initial MAD model estimator
    //�Դ���������ͼ�񶼻��õ� û��ͼ������
    MADModelEstimator (n_windowSize);//���Իع��Ż� ƽ������С �õ��Ż�����
    
    // remove outlier 
    
    for (i = 0; i < (int) n_windowSize; i++) {
		//MADPictureC1�൱��w MADPictureC2�൱��b
	 //MADPictureC1*ReferenceMAD[i]+MADPictureC2 �ǵ�ǰ֡ͼ��MAD�����Թ���ֵ
	 //PictureMAD[i] �ǵ�ǰ֡ͼ��MAD����ʵֵ
      error[i] = MADPictureC1*ReferenceMAD[i]+MADPictureC2-PictureMAD[i];
      std += error[i] * error[i]; //ƽ������� 
    }
	//sqrt (std / n_windowSize) �ͱ�׼�ͬ(û����ƽ��ֵ) 
    threshold = (n_windowSize == 2) ? 0 : sqrt (std / n_windowSize);
    for (i = 0; i < (int) n_windowSize; i++) {
      if (fabs(error[i]) > threshold)//���Ԥ�����������ֵ ��������֡ͼ�� ��ͼ������ʹ��
        PictureRejected[i] = TRUE;
    }
    // always include the last data point
    //Ϊ�˱�֤���ݼ���Ϊ�� ��һ֡����������
    PictureRejected[0] = FALSE;
    
    // second MAD model estimator
    //������һЩ���Ƚϴ������
    MADModelEstimator (n_windowSize);
  }
}

void MADModelEstimator (int n_windowSize)
{
  int n_realSize = n_windowSize;
  int i;
  double oneSampleQ;
  double a00 = 0.0, a01 = 0.0, a10 = 0.0, a11 = 0.0, b0 = 0.0, b1 = 0.0;
  double MatrixValue;
  Boolean estimateX2 = FALSE;
  
  for (i = 0; i < n_windowSize; i++) {// find the number of samples which are not rejected
    if (PictureRejected[i])//���ĳһ֡ͼ�������� n_realSize�ͼ�һ�� ����¼����û�б�����ʹ�õ�ͼ��
      n_realSize--;
  }
  
  // default MAD model estimation results
  
  MADPictureC1 = MADPictureC2 = 0.0;//ĳһ֡ͼ��MAD����Ԥ��Ĳ���
  
  for (i = 0; i < n_windowSize; i++)  {
    if (!PictureRejected[i])//���ͼ��û�б�����
    //�ѵ��Ǽ�¼���һ֡û�б�����ͼ���MAD???
      oneSampleQ = PictureMAD[i];//�����һ��û�б����Ƶ�ͼ���MAD��ֵ��oneSampleQ���� 
  }
  for (i = 0; i < n_windowSize; i++)  {// if all non-rejected MAD are the same, take 1st order model
    if ((PictureMAD[i] != oneSampleQ) && !PictureRejected[i])//ͼ��û�б������Ҳ������һ֡�����Ƶ�ͼ��
      estimateX2 = TRUE;//estimateX2��־λ��ֵΪ��
    if (!PictureRejected[i])//���ͼ��û�б�����
    //��仰��֪�������������Ż�??? Wֵ
      MADPictureC1 += PictureMAD[i] / (ReferenceMAD[i]*n_realSize);//
  }
  
  // take 2nd order model to estimate X1 and X2
  if ((n_realSize >= 1) && estimateX2) {
    for (i = 0; i < n_windowSize; i++) {
      if (!PictureRejected[i]) {
	  	//ͼ��û�б����� �������ⲿ�־�����������
	  	//ƽ���������С���Ż� ���Իع�
        a00 = a00 + 1.0;
        a01 += ReferenceMAD[i];
        a10 = a01;
        a11 += ReferenceMAD[i]*ReferenceMAD[i];
        b0 += PictureMAD[i];
        b1 += PictureMAD[i]*ReferenceMAD[i];
      }
    }
    // solve the equation of AX = B
    MatrixValue=a00*a11-a01*a10;
    if(fabs(MatrixValue)>0.000001)//����ʽ��Ϊ0 A���� �н�
    {
    //ͨ������������
      MADPictureC2=(b0*a11-b1*a01)/MatrixValue;//b
      MADPictureC1=(b1*a00-b0*a10)/MatrixValue;//W
    }
    else//����ʽΪ0 A������
    {
    	//ֻ��һ���ؽ�
      MADPictureC2=0.0;//b
      MADPictureC1=b0/a01;//W
    }
    
  }
  if(img->type==P_SLICE)//�����P֡ ������ոռ���Ľ�����ǵ�ǰ������ 
  {
    PMADPictureC1=MADPictureC1;
    PMADPictureC2=MADPictureC2;
  }
}

/*
	Qstep=0.625*2^(QP/6)
*/
double QP2Qstep( int QP )//��֪QP�����������������Qstep
{
  int i; 
  double Qstep;
  static const double QP2QSTEP[6] = { 0.625, 0.6875, 0.8125, 0.875, 1.0, 1.125 };
  
  Qstep = QP2QSTEP[QP % 6];
  for( i=0; i<(QP/6); i++)
    Qstep *= 2;
  
  return Qstep;
}

int Qstep2QP( double Qstep )//0.625��224��Qstepӳ�䵽QP 0��51
{
  int q_per = 0, q_rem = 0;
  
  //  assert( Qstep >= QP2Qstep(0) && Qstep <= QP2Qstep(51) );
  if( Qstep < QP2Qstep(0))//ȷ�������Qstep��������Χ0.625��224
    return 0;
  else if (Qstep > QP2Qstep(51) )
    return 51;
  
  while( Qstep > QP2Qstep(5) )
  {
    Qstep /= 2;
    q_per += 1;
  }
  
  if (Qstep <= (0.625+0.6875)/2) 
  {
    Qstep = 0.625;
    q_rem = 0;
  }
  else if (Qstep <= (0.6875+0.8125)/2)
  {
    Qstep = 0.6875;
    q_rem = 1;
  }
  else if (Qstep <= (0.8125+0.875)/2)
  {
    Qstep = 0.8125;
    q_rem = 2;
  }
  else if (Qstep <= (0.875+1.0)/2)
  {
    Qstep = 0.875;
    q_rem = 3;
  }
  else if (Qstep <= (1.0+1.125)/2)
  {
    Qstep = 1.0;  
    q_rem = 4;
  }
  else 
  {
    Qstep = 1.125;
    q_rem = 5;
  }
  
  return (q_per * 6 + q_rem);
}
