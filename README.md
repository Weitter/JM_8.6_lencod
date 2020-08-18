# JM_8.6_lencod<br>
JM8.6 encoder source code is annotated into Chinese(逐行中文注释后的H.264编码器（JM8.6源码)）<br>
## 中文注释<br>
### 比如对编码过程中坐标系转换<br>
```
//得到以图像左上角为原点  该宏块左上角的16*16像素块坐标
void get_mb_block_pos (int mb_addr, int *x, int*y)
//得到以图像左上角为原点  该宏块左上角的像素坐标
void get_mb_pos (int mb_addr, int *x, int*y)
//当前宏块左上角为原点 xN，yN表示整像素偏移
void getNonAffNeighbour(unsigned int curr_mb_nr, int xN, int yN, int luma, PixelPos *pix)

```
### 比如对MAD线性回归、码率模型参数的注释<br>
```
 if ((n_realSize >= 1) && estimateX2) {
    for (i = 0; i < n_windowSize; i++) {
      if (!PictureRejected[i]) {//没有被限制使用的图像帧作为数据集
/*	这部分就是在求解参数
	平方差求和最小来优化 线性回归
	PictureMAD[i]=MADPictureC1*ReferenceMAD[i]+MADPictureC2
*/
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
    
   ```
### 关键函数说明<br>
最近会写一些博客<br>
