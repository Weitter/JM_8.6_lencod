
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

// Initiate rate control parameters 初始化码率控制变量
void rc_init_seq()
{
   double L1,L2,L3,bpp;
   int qp;
   int i;
  
   Xp=0;
   Xb=0;
   
   bit_rate=input->bit_rate;//目标码率
   frame_rate = (float)(img->framerate *(input->successive_Bframe + 1)) / (float) (input->jumpd + 1);//帧率 ??怎么算的
   PreviousBit_Rate=bit_rate;//保存数据
   
   /*compute the total number of MBs in a frame*/
   
   img->Frame_Total_Number_MB=img->height*img->width/256;//16*16=256 所有宏块数
   if(input->basicunit > img->Frame_Total_Number_MB)
     input->basicunit = img->Frame_Total_Number_MB;//每个基本单元的宏块数不可能超过总宏块数
   if(input->basicunit < img->Frame_Total_Number_MB)
     TotalNumberofBasicUnit = img->Frame_Total_Number_MB/input->basicunit;//基本单元个数
   
   MINVALUE=4.0;
   /*initialize the parameters of fluid flow traffic model*/
   
   BufferSize=bit_rate*2.56;//缓冲区大小
   CurrentBufferFullness=0;//当前缓冲区的充盈程度清零
   GOPTargetBufferLevel=CurrentBufferFullness;//??
   /*HRD consideration*/
   InitialDelayOffset=BufferSize*0.8;
   
   /*initialize the previous window size*/
   m_windowSize=0;
   MADm_windowSize=0;
   img->NumberofCodedBFrame=0;//已经编码的B帧重新计数
   img->NumberofCodedPFrame=0;//已经编码的P帧重新计数
   img->NumberofGOP=0;
   /*remaining # of bits in GOP */
   R = 0;//当前GOP中剩余的比特数
   /*control parameter */
   if(input->successive_Bframe>0)//有B帧 P帧 PBB..BBP
   {
     GAMMAP=0.25;//计算参考图像的比特数
     BETAP=0.9;
   }
   else//只有P帧IPPPPI
   {
     GAMMAP=0.5;
     BETAP=0.5;
   }
   
   /*quadratic 平方 rate-distortion model*/
   PPreHeader=0;
   
   Pm_X1=bit_rate*1.0;//RD模型参数
   Pm_X2=0.0;
   /* linear 线性 prediction model for P picture*/
   PMADPictureC1=1.0;//对D(失真度)的线性预测
   PMADPictureC2=0.0;
   
   for(i=0;i<20;i++)
   {
     Pm_rgQp[i]=0;
     Pm_rgRp[i]=0.0;
     PPictureMAD[i]=0.0;
   }
   PPictureMAD[20]=0.0;
   
   //Define the largest variation of quantization parameters
   PDuantQp=2;//量化参数最大的变化量
   
   /*basic unit 基本单元 layer rate control*/
    PAveHeaderBits1=0;
    PAveHeaderBits3=0;  
    if(TotalNumberofBasicUnit>=9)//当编码图像没有可用bits，这个是重新偏移QP的初始值 书上的判断是8
      DDquant=1;
    else
      DDquant=2;
    
    MBPerRow=input->img_width/16;//每一行的宏块数
    
    /*adaptive field/frame coding*/
    img->FieldControl=0;
    
    RC_MAX_QUANT = 51;  // clipping 限幅 QP的最大值
    RC_MIN_QUANT = 0;//clippin
    
    /*compute thei initial QP*/
	//码率除以帧率就是每一幅图像分配的bits 再除以图像像素 得到图像上平均每一个像素分配的bits
    bpp = 1.0*bit_rate /(frame_rate*img->width*img->height);
    if (img->width == 176) //QCIF图像
    {
      L1 = 0.1;
      L2 = 0.3;
      L3 = 0.6;
    }else if (img->width == 352)//CIF图像
    {
      L1 = 0.2;
      L2 = 0.6;
      L3 = 1.2;
    }else //其他图像
    {
      L1 = 0.6;
      L2 = 1.4;
      L3 = 2.4;
    }
    
    if (input->SeinitialQP==0)//对第一个GOP的第一个IDR帧QP设置
    {
      if(bpp<= L1)//如果每个像素分配的bits很少 初始化用较大的量化间距 QP
        qp = 35;
      else
        if(bpp<=L2)//bpp越小 分配的QP越小
          qp = 25;
        else
          if(bpp<=L3)
            qp  = 20;
          else
            qp =10;
          input->SeinitialQP = qp;
    }
}

// Initiate one GOP  初始化一个GOP
void rc_init_GOP(int np, int nb)
{
  Boolean Overum=FALSE;//默认没有超出预算
  int OverBits;
  int OverDuantQp;
  int AllocatedBits;
  int GOPDquant;

  /*check if the last GOP over uses its budget. If yes, the initial QP of the I frame in 
 the coming  GOP will be increased.*/

  if(R<0)//上一个GOP总的比特超出了预算
    Overum=TRUE;
  OverBits=-R;//R的相反数 超出的比特数(正表示有超出 负表示没有超出)

  /*initialize the lower bound and the upper bound for the target bits of each frame, HRD consideration*/
  LowerBound=(long)(R+bit_rate/frame_rate);//下界
  UpperBound1=(long)(R+InitialDelayOffset);//上界   InitialDelayOffset=BufferSize*0.8; BufferSize=bit_rate*2.56;

 /*compute the total number of bits for the current GOP*/ 
  //(1 + np + nb) I,P,B帧数量 即GOP内帧数量
  AllocatedBits = (int) floor((1 + np + nb) * bit_rate / frame_rate + 0.5);//分配的bits
  R +=AllocatedBits;//前一个GOP剩余的bits数加上这个GOP预测的bits
  Np  = np;
  Nb  = nb;

  OverDuantQp=(int)(8*OverBits/AllocatedBits+0.5);//??
  GOPOverdue=FALSE;
  
/*field coding*/
  img->IFLAG=1;

/*Compute InitialQp for each GOP*/
  TotalPFrame=np;//p帧数量
  img->NumberofGOP++;//GOP开始累加
  if(img->NumberofGOP==1)//如果是第一个GOP
  {
    MyInitialQp=input->SeinitialQP;//第一个GOP的IDR图像QP确定
    PreviousQp2=MyInitialQp-1; //recent change -0;
    QPLastGOP=MyInitialQp;//为计算后面的GOP的IDR图像的QP
  
  }
  else//除开第一个GOP
  {
/*adaptive field/frame coding*/
    if((input->PicInterlace==ADAPTIVE_CODING)\
      ||(input->MbInterlace))
    {
      if (img->FieldFrame == 1)//图像是帧?
      {
        img->TotalQpforPPicture += FrameQPBuffer;//把前一个P帧的QP添加到求和里面
        QPLastPFrame = FrameQPBuffer;//为计算后面的GOP的IDR图像的QP
      }
      else//图像是场?
      {
        img->TotalQpforPPicture += FieldQPBuffer;
        QPLastPFrame = FieldQPBuffer;
      }
      
    }
    /*compute the average QP of P frames in the previous GOP*/
    PAverageQp=(int)(1.0*img->TotalQpforPPicture/img->NumberofPPicture+0.5);//前一个GOP中所有P帧的平均QP 

    GOPDquant=(int)(0.5+1.0*(np+nb+1)/15);//偏移 p166
    if(GOPDquant>2)
        GOPDquant=2;//限幅

    PAverageQp-=GOPDquant;
	//I帧之前肯定是最后一个P帧
    if (PAverageQp > (QPLastPFrame - 2))//QPLastPFrame是前一个GOP最后一个P帧图像的QP
      PAverageQp--;
    PAverageQp = MAX(QPLastGOP-2,  PAverageQp);//QPLastGOP是上一个GOP的IDR图像的QP
    PAverageQp = MIN(QPLastGOP+2, PAverageQp);
    PAverageQp = MIN(RC_MAX_QUANT, PAverageQp);//RC_MAX_QUANT是51
    PAverageQp = MAX(RC_MIN_QUANT, PAverageQp);//最低是0
  

    MyInitialQp=PAverageQp;//非第一个GOP中IDR帧初始化的QP 
    QPLastGOP = MyInitialQp;//保存起来 为求解下一个GOP的IDR帧QP做准备
    Pm_Qp=PAverageQp;
    PAveFrameQP=PAverageQp;
    PreviousQp1=PreviousQp2;
    PreviousQp2=MyInitialQp-1;  
  }

  img->TotalQpforPPicture=0;//上面已经使用过该数据 这个地方清零 当前帧的数据求出了后再赋值
  img->NumberofPPicture=0;
  NumberofBFrames=0; 
}

void rc_init_pict(int fieldpic,int topfield,int targetcomputation)
{
  int i;

/*compute the total number of basic units in a frame*/
  if(input->MbInterlace)
    TotalNumberofBasicUnit=img->Frame_Total_Number_MB/img->BasicUnit;//基本单元的个数 基本单元由若干个宏块构成
  img->NumberofCodedMacroBlocks=0;

/*Normally, the bandwith for the VBR case is estimated by 
a congestion(拥塞) control algorithm. A bandwidth curve can be predefined if we only want to 
test the proposed(建议) algorithm*/
  if(input->channel_type==1)//时变信道
  {
    if(img->NumberofCodedPFrame==58)//不是很懂?? 码率突然增大 是在程序里面测试可变码率控制??
      bit_rate *=1.5;
    else if(img->NumberofCodedPFrame==59)
      PreviousBit_Rate=bit_rate;
  }

/*predefine a target buffer level for each frame*/
  if((fieldpic||topfield)&&targetcomputation)
  {
    switch (img->type)
    {
      case P_SLICE:  //P片 在码率控制里面算法用得最多的地方
/*Since the available bandwidth may vary at any time, the total number of 
bits is updated picture by picture 每帧图像都更新R */
        if(PreviousBit_Rate!=bit_rate)//如果不是恒定码率
          R +=(int) floor((bit_rate-PreviousBit_Rate)*(Np+Nb)/frame_rate+0.5);//
              
/* predefine the  target buffer level for each picture.
frame layer rate control*/
        if(img->BasicUnit==img->Frame_Total_Number_MB)//一个基本单元是所有宏块 相当于图像级码率控制
        {
          if(img->NumberofPPicture==1)//当前GOP第一个P帧
          {
            TargetBufferLevel=CurrentBufferFullness;//Si(2)=Vi(2) 缓冲区充盈程度赋值给缓冲区级别
            DeltaP=(CurrentBufferFullness-GOPTargetBufferLevel)/(TotalPFrame-1);//书上没有减GOPTargetBufferLevel
            TargetBufferLevel -=DeltaP;
          }
          else if(img->NumberofPPicture>1)//当前GOP非第一个P帧
            TargetBufferLevel -=DeltaP;
        }
/*basic unit layer rate control*/
        else//一个基本单元是几个或一个宏块 基本单元层码率控制
        {
          if(img->NumberofCodedPFrame>0)
          {
            /*adaptive frame/filed coding*/
            if(((input->PicInterlace==ADAPTIVE_CODING)||(input->MbInterlace))\
              &&(img->FieldControl==1))
            {
              for(i=0;i<TotalNumberofBasicUnit;i++)
                FCBUPFMAD[i]=FCBUCFMAD[i];//上一个基本单元同位置的MADS来预测当前的MADS
            }
            else
            {
              for(i=0;i<TotalNumberofBasicUnit;i++)
                BUPFMAD[i]=BUCFMAD[i];
            }     
          }
			//其实GOP的判断可以去掉 和上面一样
          if(img->NumberofGOP==1)//第一个GOP
          {
            if(img->NumberofPPicture==1)//第一个GOP第一个P帧
            {
              TargetBufferLevel=CurrentBufferFullness;//和图像层码率控制一样 Si(2)=Vi(2) 
              DeltaP=(CurrentBufferFullness-GOPTargetBufferLevel)/(TotalPFrame-1);
              TargetBufferLevel -=DeltaP;
            }
            else if(img->NumberofPPicture>1)//第一个GOP后面的P帧
              TargetBufferLevel -=DeltaP;
          }
          else if(img->NumberofGOP>1)//非第一个GOP
          {
            if(img->NumberofPPicture==0)//后面GOP的第一个P帧
            {
              TargetBufferLevel=CurrentBufferFullness;
              DeltaP=(CurrentBufferFullness-GOPTargetBufferLevel)/TotalPFrame;
              TargetBufferLevel -=DeltaP;
            }
            else if(img->NumberofPPicture>0)//后面GOP的非第一个P帧
              TargetBufferLevel -=DeltaP;
          }
        }

        if(img->NumberofCodedPFrame==1)//每一个GOP的第一个P帧
          AWp=Wp;//Wp是P帧图像的复杂度 AWp是P帧图像的平均复杂度
        if((img->NumberofCodedPFrame<8)&&(img->NumberofCodedPFrame>1))//每一个GOP的第二个P帧到第七个P帧
            AWp=Wp*(img->NumberofCodedPFrame-1)/img->NumberofCodedPFrame+\
              AWp/img->NumberofCodedPFrame;
          else if(img->NumberofCodedPFrame>1)//这个判断有点怪??
            AWp=Wp/8+7*AWp/8;//书上有公式
          
        //compute the average complexity of B frames
        if(input->successive_Bframe>0)//如果存在B帧 IPBP
        {
          //compute the target buffer level  
          //TargetBufferLevel与B帧数量L(input->successive_Bframe)有关
          TargetBufferLevel +=(AWp*(input->successive_Bframe+1)*bit_rate\
            /(frame_rate*(AWp+AWb*input->successive_Bframe))-bit_rate/frame_rate);//这个时候TargetBufferLevel才真正求出来了
        }
        
        break;
		//在图像层 作为非参考帧 PBB..BP通过相邻的P帧的QP来求中间B帧的QP 
		//在基本编码单元层 所有图像单元使用相同的QP
         case B_SLICE: 	
         /* update the total number of bits if the bandwidth is changed*/
           if(PreviousBit_Rate!=bit_rate)//可变码率控制
             R +=(int) floor((bit_rate-PreviousBit_Rate)*(Np+Nb)/frame_rate+0.5);//图像层分配的bits
            if((img->NumberofCodedPFrame==1)&&(img->NumberofCodedBFrame==1))//当前GOP里面第一个B帧
          {
            AWp=Wp;//Wp是P帧图像的复杂度 AWp是P帧图像的平均复杂度
            AWb=Wb;//Wp是B帧图像的复杂度 AWp是B帧图像的平均复杂度
          }
          else if(img->NumberofCodedBFrame>1)//当前GOP里面后面的B帧
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
    if(img->type==P_SLICE)//计算P帧图像的目标bits
    {
      /*frame layer rate control*/
      if(img->BasicUnit==img->Frame_Total_Number_MB)//一个基本单元所有宏块 相当于图像层编码控制
      {
        if(img->NumberofCodedPFrame>0)
        {
        //从当前GOP可用bit的角度来计算
          T = (long) floor(Wp*R/(Np*Wp+Nb*Wb) + 0.5);//带有权重的计算平均一帧参考图像需要的bits
          //从目标码率 帧率 缓冲区占用情况 目标缓存级别
          T1 = (long) floor(bit_rate/frame_rate-GAMMAP*(CurrentBufferFullness-TargetBufferLevel)+0.5);
          T1=MAX(0,T1);
          T = (long)(floor(BETAP*T+(1.0-BETAP)*T1+0.5));
        }
       }
      /*basic unit layer rate control*/
      else//基本层的码率控制
      {
        if((img->NumberofGOP==1)&&(img->NumberofCodedPFrame>0))//第一个GOP
        {
          T = (int) floor(Wp*R/(Np*Wp+Nb*Wb) + 0.5);
          T1 = (int) floor(bit_rate/frame_rate-GAMMAP*(CurrentBufferFullness-TargetBufferLevel)+0.5);
          T1=MAX(0,T1);
          T = (int)(floor(BETAP*T+(1.0-BETAP)*T1+0.5));
        }
        else if(img->NumberofGOP>1)//后面的GOP
        {
          T = (long) floor(Wp*R/(Np*Wp+Nb*Wb) + 0.5);
          T1 = (long) floor(bit_rate/frame_rate-GAMMAP*(CurrentBufferFullness-TargetBufferLevel)+0.5);
          T1 = MAX(0,T1);
          T = (long)(floor(BETAP*T+(1.0-BETAP)*T1+0.5));
        }
      }

      /*reserve some bits for smoothing*/

      T=(long)((1.0-0.0*input->successive_Bframe)*T);//0乘?? 这句话有问题吧?????
      /*HRD consideration*/
      T = MAX(T, (long) LowerBound);//可变信道的最低带宽
        T = MIN(T, (long) UpperBound2);//可变信道的最高带宽

      if((topfield)||(fieldpic&&((input->PicInterlace==ADAPTIVE_CODING)\
        ||(input->MbInterlace))))
        T_field=T;
    }
  }

  if(fieldpic||topfield)//顶场
  {
    /*frame layer rate control*/
    img->NumberofHeaderBits=0;
    img->NumberofTextureBits=0;

    /*basic unit layer rate control*/
    if(img->BasicUnit < img->Frame_Total_Number_MB)//一个基本单元有几个宏块 基本单元层编码控制
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
    &&(img->FieldControl==1)) //P帧 场模式 基本单元层编码控制
  {
  /*top filed at basic unit layer rate control*/
    if(topfield)//顶场 
    {
      bits_topfield=0;
      T=(long)(T_field*0.6);//顶场占整个帧的所有bits的0.6
    }
  /*bottom filed at basic unit layer rate control*/
    else//底场
    {
      T=T_field-bits_topfield;//底场占整个帧的所有bits的0.4
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
      s+= abs(diffy[k][l]);//残差矩阵里面每一个元素绝对值求和
  
  MAD=s*1.0/256;//因为是平均绝对误差 所有要除以16*16
  return MAD;
}

// update one picture after frame/field encoding
void rc_update_pict(int nbits)// nbits是前一图像编码后的实际比特数
{
//这个地方一幅图像分配的bits才真正算完
  R-= nbits; /* remaining # of bits in GOP 当前GOP的剩余bits要减去上一帧实际的产生的bits(根据码率控制计算的QP量化编码后的比特数)
  */
  //当前缓冲区的充盈程度 根据漏斗原则             		nbits - bit_rate/frame_rate表示净增加的bits
  CurrentBufferFullness += nbits - bit_rate/frame_rate;
	
  /*update the lower bound and the upper bound for the target bits of each frame, HRD consideration*/
  LowerBound  +=(long)(bit_rate/frame_rate-nbits);//(bit_rate/frame_rate-nbits)表示净输出的Bits
  UpperBound1 +=(long)(bit_rate/frame_rate-nbits);
  UpperBound2 = (long)(OMEGA*UpperBound1);
  
  return;
}

// update after frame encoding 编码接收后对已编码B P帧的计数器累加，获取编码头部信息，更新图像复杂度
void rc_update_pict_frame(int nbits)
{

/*update the
complexity weight of I, P, B frame*/
  int Avem_Qc;
  int X;
    
/*frame layer rate control*/
  if(img->BasicUnit==img->Frame_Total_Number_MB)//图像层码率控制
    X = (int) floor(nbits*m_Qc+ 0.5);//计算复杂度??
/*basic unit layer rate control*/
  else//基本编码单元层码率控制
  {
    if(img->type==P_SLICE)//P帧
    {
      if(((img->IFLAG==0)&&(img->FieldControl==1))\
        ||(img->FieldControl==0))
      {
        Avem_Qc=TotalFrameQP/TotalNumberofBasicUnit;//一幅P图像里面的平均QP，由一幅P图像里面每一个编码单元求平均计算得到
        X=(int)floor(nbits*Avem_Qc+0.5);
      }
    }
    else if(img->type==B_SLICE)//B帧
      X = (int) floor(nbits*m_Qc+ 0.5);
  }


  switch (img->type)
  {
     //P帧
    case P_SLICE:
 /*filed coding*/
      if(((img->IFLAG==0)&&(img->FieldControl==1))\
        ||(img->FieldControl==0))
      {
        Xp = X;
        Np--;//NP减少 说明NP是表示当前GOP中剩余的P帧数
        Wp=Xp;//计算P帧图像复杂度 与图像实际产生的比特数和平均QP有关
        Pm_Hp=img->NumberofHeaderBits;//头部bit
        img->NumberofCodedPFrame++;
        img->NumberofPPicture++;
      }
      else if((img->IFLAG!=0)&&(img->FieldControl==1))
        img->IFLAG=0;
        break;
        case B_SLICE://B帧
        Xb = X;
        Nb--;
        Wb=Xb/THETA;// 计算B帧图像复杂度 与图像实际产生的比特数和平均QP有关
    
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
//计算每一帧的量化参数	QP      	得到码率控制的最终结果 
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
  if(img->BasicUnit==img->Frame_Total_Number_MB)//基于图像层的码率控制
  {
/*fixed quantization parameter is used to coded I frame, the first P frame and the first B frame
  the quantization parameter is adjusted according the available channel bandwidth and 
  the type of vide*/  
/*top field*/
    if((topfield)||(img->FieldControl==0))//顶场
    {
      if(img->type==I_SLICE)//图像是I帧
      {
        m_Qc=MyInitialQp;//GOP的第一帧IDR图像QP 
        return m_Qc;
      }
      else if(img->type==B_SLICE)//B片
      {
        if(input->successive_Bframe==1)//一个GOP只有一个B帧 PBP
        {
        //PBB..BBBP BFrameNumber是当前编码帧结构里面的B帧顺序 12...56
          BFrameNumber=(NumberofBFrames+1)%input->successive_Bframe;
          if(BFrameNumber==0)
            BFrameNumber=input->successive_Bframe;
          /*adaptive field/frame coding*/
          else if(BFrameNumber==1)//当前一小段(两个P帧间)里面的第一个B帧
          {
            if((input->PicInterlace==ADAPTIVE_CODING)\
              ||(input->MbInterlace))//采取交织
            {
              if(img->FieldControl==0)
              {                   
                /*previous choice is frame coding*/
                if(img->FieldFrame==1)//帧
                {
                //PreviousQp2 B FrameQPBuffer
                //PreviousQp1 B PreviousQp2 B FrameQPBuffer
                //			    PreviousQp1 B PreviousQp2
                  PreviousQp1=PreviousQp2;//PBP里面左边的P帧QP
                  PreviousQp2=FrameQPBuffer;//PBP里面右边的P帧QP
                }           
                /*previous choice is field coding*/
                else//场
                {
                  PreviousQp1=PreviousQp2;
                  PreviousQp2=FieldQPBuffer;
                }
              }
            }
          }
          if(PreviousQp1==PreviousQp2)//PBP的结构 如果前后P帧的量化参数一样 B帧的量化参数加2
            m_Qc=PreviousQp1+2;
          else////PBP的结构 如果前后P帧的量化参数不一样 为了保证平滑 按照下面算
            m_Qc=(PreviousQp1+PreviousQp2)/2+1;
          m_Qc = MIN(m_Qc, RC_MAX_QUANT); // clipping
          m_Qc = MAX(RC_MIN_QUANT, m_Qc);//clipping
        }
        else//一个GOP只有多个B帧 PBB..BP
        {
         //PBB..BBBP BFrameNumber是当前编码帧结构里面的B帧顺序 12...56
          BFrameNumber=(NumberofBFrames+1)%input->successive_Bframe;
          if(BFrameNumber==0)
            BFrameNumber=input->successive_Bframe;
          /*adaptive field/frame coding*/
          else if(BFrameNumber==1)//当前一小段(两个P帧间)里面的第一个B帧
          {
            if((input->PicInterlace==ADAPTIVE_CODING)\
              ||(input->MbInterlace))
            {
              if(img->FieldControl==0)
              {
                /*previous choice is frame coding*/
                if(img->FieldFrame==1)
                {
                  PreviousQp1=PreviousQp2;//PBB...BP里面左边的P帧QP
                  PreviousQp2=FrameQPBuffer;//PBB...BP里面右边的P帧QP
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
            (BFrameNumber-1)*(PreviousQp2-PreviousQp1)/(input->successive_Bframe-1)));//多乘了(BFrameNumber-1)
          m_Qc = MIN(m_Qc, RC_MAX_QUANT); // clipping 限幅
          m_Qc = MAX(RC_MIN_QUANT, m_Qc);//clipping
        }
        return m_Qc;
      }
      else if((img->type==P_SLICE)&&(img->NumberofPPicture==0))//第一个IDR帧(I或P)
      {
        m_Qc=MyInitialQp;//好像直接用的I帧的QP来用的
        
        if(img->FieldControl==0)
        {
          if(active_sps->frame_mbs_only_flag)
          {
            img->TotalQpforPPicture +=m_Qc;//保存当前的QP 累加到所有P帧QP和的变量里面
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
      else//后面的P帧
      {
      /*
		第一步:确定每一个参考图像(P)的目标目标bits
		第二步:确定参考图像的QP 运行RDO优化
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
            Pm_Qp=FieldQPBuffer;//前一帧P帧的QP
          }
        }
		//Pm_X1=bit_rate*1.0;//RD模型参数
   		//Pm_X2=0.0;
        m_X1=Pm_X1;//RD模型的两个参数
        m_X2=Pm_X2;
        m_Hp=PPreHeader;
        m_Qp=Pm_Qp;//前一帧P帧的QP
        DuantQp=PDuantQp;//量化参数最大的变化量 2
        MADPictureC1=PMADPictureC1;//PMADPictureC1=1 线性预测的两个参数
        MADPictureC2=PMADPictureC2;//PMADPictureC1=0
        PreviousPictureMAD=PPictureMAD[0];
        
        /* predict the MAD of current picture*/
        CurrentFrameMAD=MADPictureC1*PreviousPictureMAD+MADPictureC2;//线性预测当前帧的MAD
        
        /*compute the number of bits for the texture*/      
        
        if(T<0)//当T小于0 直接使用前一个P帧的QP来预测 然后加2 得到当前的QP值
        {
          m_Qc=m_Qp+DuantQp;//前一帧P帧的QP加2 DuantQp=2
          m_Qc = MIN(m_Qc, RC_MAX_QUANT); // clipping
        }
        else//当T大于0 执行RD优化
        {
          m_Bits =T-m_Hp;//减去头部bits ??
          m_Bits = MAX(m_Bits, (int)(bit_rate/(MINVALUE*frame_rate)));//MINVALUE=4 确定一个最大值上界
          //求解m_Bits(Qstep)=X1*D/Qstep+x2*D/(Qstep*Qstep)一元二次方程解
          dtmp = CurrentFrameMAD * m_X1 * CurrentFrameMAD * m_X1 \
            + 4 * m_X2 * CurrentFrameMAD * m_Bits;//判别式
          if ((m_X2 == 0.0) || (dtmp < 0) || ((sqrt (dtmp) - m_X1 * CurrentFrameMAD) <= 0.0)) // fall back 1st order mode
            m_Qstep = (float) (m_X1 * CurrentFrameMAD / (double) m_Bits);
          else // 2nd order mode
            m_Qstep = (float) ((2 * m_X2 * CurrentFrameMAD) / (sqrt (dtmp) - m_X1 * CurrentFrameMAD));
          
          m_Qc=Qstep2QP(m_Qstep);//0.625到224的Qstep映射到QP 0到51
          //m_Qp是前一P帧QP的预测
          m_Qc = MIN(m_Qp+DuantQp,  m_Qc);  // control variation
          m_Qc = MIN(m_Qc, RC_MAX_QUANT); // clipping
          m_Qc = MAX(m_Qp-DuantQp, m_Qc); // control variation
          m_Qc = MAX(RC_MIN_QUANT, m_Qc);//	QP平滑和限幅
        }
        
        if(img->FieldControl==0)
        {
          /*frame coding*/
          if(active_sps->frame_mbs_only_flag)
          {
            img->TotalQpforPPicture +=m_Qc;//记录累加现在的QP 求解下一个GOP的IDR帧的QP
            PreviousQp1=PreviousQp2;//记录相邻P帧的QP 为了计算B帧QP
            PreviousQp2=m_Qc;
            Pm_Qp=m_Qc;//当前P帧的QP对下一帧P帧预测
          }
          /*adaptive field/frame coding*/
          else
            FrameQPBuffer=m_Qc;
        }
        
        return m_Qc;
      }
   }
   /*bottom field*/
   else//底场
   {
     if((img->type==P_SLICE)&&(img->IFLAG==0))
     {
       /*field coding*/
       if(input->PicInterlace==FIELD_CODING)
       {
         img->TotalQpforPPicture +=m_Qc;//从什么地方算的??
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
  else //基本编码单元层的码率控制        		I帧和P帧都和前面图像层码率控制一样
  {
    /*top filed of I frame*/
    if(img->type==I_SLICE)//I帧内所有编码单元的宏块QP相同
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
    else if(img->type==B_SLICE)//B帧
    {
      /*top filed of B frame*/
      if((topfield)||(img->FieldControl==0))//B帧顶场
      {
        if(input->successive_Bframe==1)//PBP 只有一个B
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
        else//PBB...BP形式
        {
        /*
			比如PBBBBP    input->successive_Bframe=4
			NumberofBFrames=0  BFrameNumber=1
			NumberofBFrames=1  BFrameNumber=2
			NumberofBFrames=2  BFrameNumber=3
			NumberofBFrames=3  BFrameNumber=0
			BFrameNumber是一个帧结构内部的B帧顺序
		*/
          BFrameNumber=(NumberofBFrames+1)%input->successive_Bframe;
        if(BFrameNumber==0)
          BFrameNumber=input->successive_Bframe;
        /*adaptive field/frame coding*/
        else if(BFrameNumber==1)//PBBBP的第一个B
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
    else if(img->type==P_SLICE)//P帧内基本编码单元层的码率控制
    {
      if((img->NumberofGOP==1)&&(img->NumberofPPicture==0))//第一个GOP的第一个P帧
      {
        if((img->FieldControl==0)||((img->FieldControl==1)\
          &&(img->IFLAG==0)))
        {
        /*top field of the first P frame*/
          m_Qc=MyInitialQp;//和I帧一样 使用fix的方法
          img->NumberofBasicUnitHeaderBits=0;
          img->NumberofBasicUnitTextureBits=0;
          NumberofBasicUnit--;//基本单元数量逐减
        /*bottom field of the first P frame*/
          if((!topfield)&&(NumberofBasicUnit==0))//底场的第一个基本单元
          {
            /*frame coding or field coding*/
            if((active_sps->frame_mbs_only_flag)||(input->PicInterlace==FIELD_CODING))//帧或者场
            {
              img->TotalQpforPPicture +=m_Qc;
              PreviousQp1=PreviousQp2;
              PreviousQp2=m_Qc;
              PAveFrameQP=m_Qc;//保存当前的QP ??
              PAveHeaderBits3=PAveHeaderBits2;//头部信息bits??
            }
            /*adaptive frame/field coding*/
            else if((input->PicInterlace==ADAPTIVE_CODING)\
              ||(input->MbInterlace))//交织
            {
              if(img->FieldControl==0)//顶场
              {
                FrameQPBuffer=m_Qc;
                FrameAveHeaderBits=PAveHeaderBits2;
              }
              else//底场
              {
                FieldQPBuffer=m_Qc;
                FieldAveHeaderBits=PAveHeaderBits2;
              }
            }
          }
          Pm_Qp=m_Qc;//给后面预测
          TotalFrameQP +=m_Qc;//当前帧内的QP数累加
          return m_Qc;
        }
      }
      else// 非第一帧的P帧 使用RD优化算法
      {
        m_X1=Pm_X1;//RD优化的两个参数
        m_X2=Pm_X2;
        m_Hp=PPreHeader;//头部信息
        m_Qp=Pm_Qp;//预测QP
        DuantQp=PDuantQp;//QP的偏移量2
        MADPictureC1=PMADPictureC1;//MAD线性预测的参数
        MADPictureC2=PMADPictureC2;

        if(img->FieldControl==0)//底场
          SumofBasicUnit=TotalNumberofBasicUnit;
        else//顶场
          SumofBasicUnit=TotalNumberofBasicUnit/2;
		/*
		TotalNumberofBasicUnit:是总的基本编码单元数
		NumberofBasicUnit:说明这个值最大是第一个编码单元 为0是最后一个编码单元
		*/
        /*the average QP of the previous frame is used to coded the first basic unit of the current frame or field*/
        if(NumberofBasicUnit==SumofBasicUnit)//当编码到第一个编码单元 看不出在求平均
        {

          /*adaptive field/frame coding*/
          if(((input->PicInterlace==ADAPTIVE_CODING)\
            ||(input->MbInterlace))\
            &&(img->FieldControl==0))//交织且底场
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

          if(T<=0)//如果分配的bits不够用 
          {
            m_Qc=PAveFrameQP+2;//增大QP
            if(m_Qc>RC_MAX_QUANT)//限幅
              m_Qc=RC_MAX_QUANT;
            if(topfield||(img->FieldControl==0))
              GOPOverdue=TRUE;//这个标志位在干啥??
          }
          else//如果图像分配的bits够用
          {
            m_Qc=PAveFrameQP; //不用预先增加QP
          }
          TotalFrameQP +=m_Qc;
          NumberofBasicUnit--;
          Pm_Qp=PAveFrameQP;
          return m_Qc;
        }
		else//当编码到非最后一个编码单元
         {
          /*compute the number of remaining bits 计算当前图像剩余bits*/
           TotalBasicUnitBits=img->NumberofBasicUnitHeaderBits+img->NumberofBasicUnitTextureBits;//当前编码单元头部和编码内容bits????
           T -=TotalBasicUnitBits;//当前帧图像分配的bits前去上一个编码单元实际用的bits
           img->NumberofBasicUnitHeaderBits=0;//清空
           img->NumberofBasicUnitTextureBits=0;
           if(T<0)//如果预测的bits数不够用了
           {
             if(GOPOverdue==TRUE)//调整QP 使其变大
               m_Qc=m_Qp+2;
             else 
               m_Qc=m_Qp+DDquant;//2 
             m_Qc = MIN(m_Qc, RC_MAX_QUANT);  // clipping  限幅
             if(input->basicunit>=MBPerRow)//基本单元数大于每一行的宏块数
               m_Qc = MIN(m_Qc, PAveFrameQP+6); 
             else
               m_Qc = MIN(m_Qc, PAveFrameQP+3);

             TotalFrameQP +=m_Qc;//累加到求和帧QP里面
             NumberofBasicUnit--;//是递减 说明这个值最大是第一个编码单元 为0是最后一个编码单元
             if(NumberofBasicUnit==0)//最后编码单元
             {
               if((!topfield)||(img->FieldControl==0))
               {
                 /*frame coding or field coding*/
                 if((active_sps->frame_mbs_only_flag)||(input->PicInterlace==FIELD_CODING))
                 {
                   PAverageQP=(int)(1.0*TotalFrameQP/TotalNumberofBasicUnit+0.5);//得到帧内平均QP
                   if (img->NumberofPPicture == (input->intra_period - 2))//不是很懂???
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
           else//图像分配的bits够用 使用优化算法
           {
             /*predict the MAD of current picture*/
             if(((input->PicInterlace==ADAPTIVE_CODING)||(input->MbInterlace))\
               &&(img->FieldControl==1))
             {
             //线性预测当前的MAD 但是这个地方找的预测值隔得好远???
               CurrentFrameMAD=MADPictureC1*FCBUPFMAD[TotalNumberofBasicUnit-NumberofBasicUnit]+MADPictureC2;
               TotalBUMAD=0;
			   //TotalNumberofBasicUnit=10  NumberofBasicUnit=3(第7个编码单元) 
			   //TotalNumberofBasicUnit-NumberofBasicUnit是目前第几个编码单元
               for(i=TotalNumberofBasicUnit-1; i>=(TotalNumberofBasicUnit-NumberofBasicUnit);i--)
               {
                 CurrentBUMAD=MADPictureC1*FCBUPFMAD[i]+MADPictureC2;//前面已经编码的编码单元
                 TotalBUMAD +=CurrentBUMAD*CurrentBUMAD;//前面已经编码的编码单元的平方的求和
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
			 //通过MAD的占比来分配bits MAD(失真度)越大 需要用更多的bits来优化QP
             m_Bits =(int)(T*CurrentFrameMAD*CurrentFrameMAD/TotalBUMAD);
             /*compute the number of texture bits*/
             m_Bits -=PAveHeaderBits2;//头部bits
             
             m_Bits=MAX(m_Bits,(int)(bit_rate/(MINVALUE*frame_rate*TotalNumberofBasicUnit)));//和P帧一样，确定一个最大的分配比特数

			 //求解一元二次方程的判别式 使用RD优化的参数一样
             dtmp = CurrentFrameMAD * m_X1 * CurrentFrameMAD * m_X1 \
               + 4 * m_X2 * CurrentFrameMAD * m_Bits;
             if ((m_X2 == 0.0) || (dtmp < 0) || ((sqrt (dtmp) - m_X1 * CurrentFrameMAD) <= 0.0))  // fall back 1st order mode
               m_Qstep = (float)(m_X1 * CurrentFrameMAD / (double) m_Bits);
             else // 2nd order mode
               m_Qstep = (float) ((2 * m_X2 * CurrentFrameMAD) / (sqrt (dtmp) - m_X1 * CurrentFrameMAD));

             m_Qc=Qstep2QP(m_Qstep);//从0.625到224 映射到0到51
             m_Qc = MIN(m_Qp+DDquant,  m_Qc); // 预测QP加上偏移来控制当前的QP范围control variation

             if(input->basicunit>=MBPerRow)//继续调整
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
             TotalFrameQP +=m_Qc;//记录现在算出来的QP 加到累加器里
             Pm_Qp=m_Qc;//把现在的QP作为下一个的预测
             NumberofBasicUnit--;//编码单元数递减
             if((NumberofBasicUnit==0)&&(img->type==P_SLICE))//P帧 且编码单元数减到0 就最后一个编码单元
             {
               if((!topfield)||(img->FieldControl==0))//底场
               {
                 /*frame coding or field coding*/
                 if((active_sps->frame_mbs_only_flag)||(input->PicInterlace==FIELD_CODING))
                 {
                   PAverageQP=(int)(1.0*TotalFrameQP/TotalNumberofBasicUnit+0.5);//计算平均QP
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
void updateRCModel ()//更新模型的参数
{

  int n_windowSize;
  int i;
  double error[20], std = 0.0, threshold;
  int m_Nc;
  Boolean MADModelFlag = FALSE;
   
  if(img->type==P_SLICE)//当前是P帧
  {
    /*
    frame layer rate control
  */
    if(img->BasicUnit==img->Frame_Total_Number_MB)//当前是图像层的码率控制
    {
      CurrentFrameMAD=ComputeFrameMAD();//得到当前图像帧的MAD值
      m_Nc=img->NumberofCodedPFrame;//已经编码的P帧数量
    }
    /*basic unit layer rate control*/
    else//对于基本编码单元的码率控制
    {
      /*compute the MAD of the current basic unit*/
	//记录当前基本编码单元的MAD
      if((input->MbInterlace)&&(img->FieldControl==0))//交织底场
        CurrentFrameMAD=img->TotalMADBasicUnit/img->BasicUnit/2;
      else
        CurrentFrameMAD=img->TotalMADBasicUnit/img->BasicUnit;//基本编码单元的MAD是一个平均值

        
      img->TotalMADBasicUnit=0;//用完这个数据就释放
              
      /* compute the average number of header bits*/
      
        CodedBasicUnit=TotalNumberofBasicUnit-NumberofBasicUnit;//已经编码的编码单元的排序
        //NumberofBasicUnit=TotalNumberofBasicUnit-CodedBasicUnit 书上170页编码单元头部信息估计
        if(CodedBasicUnit>0)
        {
        //书上有公式 对编码单元头部的预测
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
          &&(img->FieldControl==1))//交织和顶场
            FCBUCFMAD[TotalNumberofBasicUnit-1-NumberofBasicUnit]=CurrentFrameMAD;//存储当前的MAD
          else
            BUCFMAD[TotalNumberofBasicUnit-1-NumberofBasicUnit]=CurrentFrameMAD;

        if(NumberofBasicUnit!=0)//没有到最后一个编码单元
          m_Nc=img->NumberofCodedPFrame*TotalNumberofBasicUnit+CodedBasicUnit;//这个是当前GOP内现在的编码单元的顺序
        else
          m_Nc=(img->NumberofCodedPFrame-1)*TotalNumberofBasicUnit+CodedBasicUnit;
      
    }
    
    
   
    if(m_Nc>1)
      MADModelFlag=TRUE;
    PPreHeader=img->NumberofHeaderBits;//头部bits
    /*
	在
	*/
    for (i = 19; i > 0; i--) {// update the history 对搜索窗口的遍历
    // Pm_rgQp[18] Pm_rgQp[17] ...... Pm_rgQp[0]
    // Pm_rgQp[19] Pm_rgQp[18] .....  Pm_rgQp[1] m_rgQp[0]=Pm_rgQp[0];
      Pm_rgQp[i] = Pm_rgQp[i - 1];//数据在移位 腾出下标为0的地方 放当前的Qstep预测值
      m_rgQp[i]=Pm_rgQp[i];//把预测值赋值
	//对Pm_rgRp也同样操作
      Pm_rgRp[i] = Pm_rgRp[i - 1];
      m_rgRp[i]=Pm_rgRp[i];
    }
    Pm_rgQp[0] = QP2Qstep(m_Qc); //*1.0/CurrentFrameMAD; 计算当前QP的Qstep
    /*frame layer rate control*/
    if(img->BasicUnit==img->Frame_Total_Number_MB)//图像层码率控制
    //img->NumberofTextureBits是图像层实际使用的bits
      Pm_rgRp[0] = img->NumberofTextureBits*1.0/CurrentFrameMAD;//编码的bits除以MAD ??想表达啥 是替代后面优化的标记数据
    /*basic unit layer rate control*/
    else//基本单元层码率控制
    //img->NumberofBasicUnitTextureBits是基本编码单元层实际使用的bits
      Pm_rgRp[0]=img->NumberofBasicUnitTextureBits*1.0/CurrentFrameMAD;//

    m_rgQp[0]=Pm_rgQp[0];//QP的Qstep预测值赋值 xi
    //如果该帧图像分配的bits是R(i),或者叫T(i),
    //m_rgRp[i]=R(i)/CurrentFrameMAD 所以m_rgRp[i]不是真正的该帧图像分配的bits 做了一个映射 方便后面矩阵求解
    m_rgRp[0]=Pm_rgRp[0];// yi/Di
    m_X1=Pm_X1;
    m_X2=Pm_X2;
  

/*compute the size of window*/


    n_windowSize = (CurrentFrameMAD>PreviousFrameMAD)?(int)(PreviousFrameMAD/CurrentFrameMAD*20)\
      :(int)(CurrentFrameMAD/PreviousFrameMAD*20);
    n_windowSize=MAX(n_windowSize, 1);
    n_windowSize=MIN(n_windowSize,m_Nc);//确定窗口的上界m_Nc 防止计算范围超出去了一大截
    n_windowSize=MIN(n_windowSize,m_windowSize+1);//m_windowSize是上一次使用的窗口 防止窗口阶跃跳动太大 但给1是不是太少了??
    n_windowSize=MIN(n_windowSize,20);

      /*update the previous window size*/
  m_windowSize=n_windowSize;
  


  for (i = 0; i < 20; i++) {//第一次参数估计时候对窗口内所有帧图像都处理
    m_rgRejected[i] = FALSE;
  }

  // initial RD model estimator
  RCModelEstimator (n_windowSize);//通过窗口内的数据集(Qstep[i],R[i]/D),RD模型的m_x1,m_x2两个参数估计
 
  n_windowSize = m_windowSize;
  // remove outlier 
  
  for (i = 0; i < (int) n_windowSize; i++) {
	//计算误差是看参数估计后模型计算结果怎么样 
    error[i] = m_X1 / m_rgQp[i] + m_X2 / (m_rgQp[i] * m_rgQp[i]) - m_rgRp[i];
    std += error[i] * error[i]; 
  }
  //sqrt (std / n_windowSize)是均方误差
  threshold = (n_windowSize == 2) ? 0 : sqrt (std / n_windowSize);
  for (i = 0; i < (int) n_windowSize; i++) {
    if (fabs(error[i]) > threshold)//对预测误差比较大的图像帧 标记为限制访问
      m_rgRejected[i] = TRUE;
  }
    // always include the last data point
  m_rgRejected[0] = FALSE;//默认第一帧图像无条件保留 防止数据集空

  // second RD model estimator
  RCModelEstimator (n_windowSize);//将预计误差比较大的图像帧剔除后 再次进行参数估计

  if(MADModelFlag)
    updateMADModel();//对图像的MAD进行线性回归估计
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

//码率控制模型的核心
void RCModelEstimator (int n_windowSize)
{
  int n_realSize = n_windowSize;
  int i;
  double oneSampleQ;
  double a00 = 0.0, a01 = 0.0, a10 = 0.0, a11 = 0.0, b0 = 0.0, b1 = 0.0;
  double MatrixValue;
  Boolean estimateX2 = FALSE;

  for (i = 0; i < n_windowSize; i++) {// find the number of samples which are not rejected
    if (m_rgRejected[i])//如果存在某一帧图像数据被拒绝
      n_realSize--;//求和遍历的n减少 即被限制的数据不参加优化过程
  }

  // default RD model estimation results

  m_X1 = m_X2 = 0.0;//码率控制模型参数先清零

  for (i = 0; i < n_windowSize; i++)  {
    if (!m_rgRejected[i])//
      oneSampleQ = m_rgQp[i];//m_rgQp[i]里面是Qstep值 记录最后一个没有被限制的数据
  }
  for (i = 0; i < n_windowSize; i++)  {// if all non-rejected Q are the same, take 1st order model
    if ((m_rgQp[i] != oneSampleQ) && !m_rgRejected[i])//如果当前的数据不是最后一个且当前数据没有被限制使用
      estimateX2 = TRUE;//标志位置为真
    if (!m_rgRejected[i])//如果数据没有被拒绝
      m_X1 += (m_rgQp[i] * m_rgRp[i]) / n_realSize;//为啥这样算???
  }

  // take 2nd order model to estimate X1 and X2 看不懂求解过程????
  if ((n_realSize >= 1) && estimateX2) {
      for (i = 0; i < n_windowSize; i++) {
      if (!m_rgRejected[i]) {//数据没有被限制使用 
      //线性回归优化
        a00 = a00 + 1.0;
        a01 += 1.0 / m_rgQp[i];
        a10 = a01;
        a11 += 1.0 / (m_rgQp[i] * m_rgQp[i]);
        b0 += m_rgQp[i] * m_rgRp[i];
        b1 += m_rgRp[i];
      }
    }
    // solve the equation of AX = B
      MatrixValue=a00*a11-a01*a10;//A矩阵行列式
      if(fabs(MatrixValue)>0.000001)//A行列式不为0 满秩
      {
        m_X1=(b0*a11-b1*a01)/MatrixValue;
        m_X2=(b1*a00-b0*a10)/MatrixValue;
      }
      else//A的行列式为0 特解
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

double ComputeFrameMAD()//计算图像帧的MAD
{
  double TotalMAD;
  int i;
  TotalMAD=0.0;
//  CurrentFrameMAD=0.0;
  for(i=0;i<img->Frame_Total_Number_MB;i++)//对当前图像帧里面的宏块逐个遍历
    TotalMAD +=img->MADofMB[i];//每一个宏块的MAD都保存在img->MADofMB[i]里面 然后累加
  TotalMAD /=img->Frame_Total_Number_MB;//得到均值
  return TotalMAD;
}


//update the parameters of linear prediction model
void updateMADModel ()
{
  
  int n_windowSize;
  int i;
  double error[20], std = 0.0, threshold;
  int m_Nc;
  
  if(img->NumberofCodedPFrame>0)//已经编完的P帧大于0
  {
    
    if(img->type==P_SLICE)//P帧
    {
      /*frame layer rate control */
      if(img->BasicUnit==img->Frame_Total_Number_MB)//图像层码率控制
        m_Nc=img->NumberofCodedPFrame;
      /*basic unit layer rate control*/
      else//基本层码率控制
        m_Nc=img->NumberofCodedPFrame*TotalNumberofBasicUnit+CodedBasicUnit;
      for (i = 19; i > 0; i--) {// update the history 最多有20个缓存值即窗口大小
      //PPictureMAD[18] PPictureMAD[17] ... PPictureMAD[0]
      //PPictureMAD[19] PPictureMAD[18] ... PPictureMAD[1] PPictureMAD[0] = CurrentFrameMAD 
      //PictureMAD[19]  PictureMAD[18]  ... PictureMAD[1]  PictureMAD[0]=PPictureMAD[0];
        PPictureMAD[i] = PPictureMAD[i - 1];//预测的MAD整体后移
        PictureMAD[i]=PPictureMAD[i];//预测的MAD给真实的MAD 用于MAD线性预测
        ReferenceMAD[i]= ReferenceMAD[i-1];//参考的MAD整体后移 用于MAD线性预测
      }
      PPictureMAD[0] = CurrentFrameMAD;//当前的MAD添加到预测MAD数组下标为0的地方
      PictureMAD[0]=PPictureMAD[0];////预测的MAD给真实的MAD 
      if(img->BasicUnit==img->Frame_Total_Number_MB)//图像层码率控制
        ReferenceMAD[0]=PictureMAD[1];//前一幅图像的MAD作为当前的参考MAD
      else//基本层码率控制
      {
        if(((input->PicInterlace==ADAPTIVE_CODING)||(input->MbInterlace))\
          &&(img->FieldControl==1))
          ReferenceMAD[0]=FCBUPFMAD[TotalNumberofBasicUnit-1-NumberofBasicUnit];
        else
          ReferenceMAD[0]=BUPFMAD[TotalNumberofBasicUnit-1-NumberofBasicUnit];
      }
      MADPictureC1=PMADPictureC1;//MAD线性预测的参数
      MADPictureC2=PMADPictureC2;
    }
    
    
    /*compute the size of window*/
    
    n_windowSize = (CurrentFrameMAD>PreviousFrameMAD)?(int)(PreviousFrameMAD/CurrentFrameMAD*20)\
      :(int)(CurrentFrameMAD/PreviousFrameMAD*20);//只有当前后图像帧的MAD近似相同的时候 表示图像平稳 采用更大的窗口
    n_windowSize=MIN(n_windowSize,(m_Nc-1));
    n_windowSize=MAX(n_windowSize, 1);
    n_windowSize=MIN(n_windowSize,MADm_windowSize+1);//前一次窗口的大小来限制当前的窗口大小
    n_windowSize=MIN(20,n_windowSize);
    /*update the previous window size*/
    MADm_windowSize=n_windowSize;//保存当前的窗口大小
    
    for (i = 0; i < 20; i++) {//所有窗口里面的图像初始化都是没有被限制使用的
      PictureRejected[i] = FALSE;
    }
    //update the MAD for the previous frame
    if(img->type==P_SLICE)//P帧
      PreviousFrameMAD=CurrentFrameMAD;//保存当前图像的MAD
    
    // initial MAD model estimator
    //对窗口内所有图像都会用到 没有图像被限制
    MADModelEstimator (n_windowSize);//线性回归优化 平方差最小 得到优化参数
    
    // remove outlier 
    
    for (i = 0; i < (int) n_windowSize; i++) {
		//MADPictureC1相当于w MADPictureC2相当于b
	 //MADPictureC1*ReferenceMAD[i]+MADPictureC2 是当前帧图像MAD的线性估计值
	 //PictureMAD[i] 是当前帧图像MAD的真实值
      error[i] = MADPictureC1*ReferenceMAD[i]+MADPictureC2-PictureMAD[i];
      std += error[i] * error[i]; //平方差求和 
    }
	//sqrt (std / n_windowSize) 和标准差不同(没有求平均值) 
    threshold = (n_windowSize == 2) ? 0 : sqrt (std / n_windowSize);
    for (i = 0; i < (int) n_windowSize; i++) {
      if (fabs(error[i]) > threshold)//如果预测的误差超过了阈值 将抛弃这帧图像 该图像被限制使用
        PictureRejected[i] = TRUE;
    }
    // always include the last data point
    //为了保证数据集不为空 第一帧无条件保留
    PictureRejected[0] = FALSE;
    
    // second MAD model estimator
    //抛弃了一些误差比较大的数据
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
    if (PictureRejected[i])//如果某一帧图像被限制了 n_realSize就减一次 最后记录的是没有被限制使用的图像
      n_realSize--;
  }
  
  // default MAD model estimation results
  
  MADPictureC1 = MADPictureC2 = 0.0;//某一帧图像MAD线性预测的参数
  
  for (i = 0; i < n_windowSize; i++)  {
    if (!PictureRejected[i])//如果图像没有被限制
    //难道是记录最后一帧没有被限制图像的MAD???
      oneSampleQ = PictureMAD[i];//把最后一个没有被限制的图像的MAD赋值给oneSampleQ变量 
  }
  for (i = 0; i < n_windowSize; i++)  {// if all non-rejected MAD are the same, take 1st order model
    if ((PictureMAD[i] != oneSampleQ) && !PictureRejected[i])//图像没有被限制且不是最后一帧被限制的图像
      estimateX2 = TRUE;//estimateX2标志位赋值为真
    if (!PictureRejected[i])//如果图像没有被限制
    //这句话不知道在做怎样的优化??? W值
      MADPictureC1 += PictureMAD[i] / (ReferenceMAD[i]*n_realSize);//
  }
  
  // take 2nd order model to estimate X1 and X2
  if ((n_realSize >= 1) && estimateX2) {
    for (i = 0; i < n_windowSize; i++) {
      if (!PictureRejected[i]) {
	  	//图像没有被限制 好厉害这部分就是在求解参数
	  	//平方差求和最小来优化 线性回归
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
    if(fabs(MatrixValue)>0.000001)//行列式不为0 A可逆 有解
    {
    //通过伴随矩阵求解
      MADPictureC2=(b0*a11-b1*a01)/MatrixValue;//b
      MADPictureC1=(b1*a00-b0*a10)/MatrixValue;//W
    }
    else//行列式为0 A不可逆
    {
    	//只是一个特解
      MADPictureC2=0.0;//b
      MADPictureC1=b0/a01;//W
    }
    
  }
  if(img->type==P_SLICE)//如果是P帧 把上面刚刚计算的结果覆盖掉前面计算的 
  {
    PMADPictureC1=MADPictureC1;
    PMADPictureC2=MADPictureC2;
  }
}

/*
	Qstep=0.625*2^(QP/6)
*/
double QP2Qstep( int QP )//已知QP求出真正的量化步长Qstep
{
  int i; 
  double Qstep;
  static const double QP2QSTEP[6] = { 0.625, 0.6875, 0.8125, 0.875, 1.0, 1.125 };
  
  Qstep = QP2QSTEP[QP % 6];
  for( i=0; i<(QP/6); i++)
    Qstep *= 2;
  
  return Qstep;
}

int Qstep2QP( double Qstep )//0.625到224的Qstep映射到QP 0到51
{
  int q_per = 0, q_rem = 0;
  
  //  assert( Qstep >= QP2Qstep(0) && Qstep <= QP2Qstep(51) );
  if( Qstep < QP2Qstep(0))//确定输入的Qstep是正常范围0.625到224
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
