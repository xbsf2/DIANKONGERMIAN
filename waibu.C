#include "waibu.h"
#include "main.h"
#include "stm32f1xx_hal.h"
static char  USART_RX_ORDER[USART_RBC_LEN]={'s','p','e','e','d','s','i','t','e','h','e','l','p','/0'};//存放指令集
unsigned int USART_RX_STA=0;
char USART_RX_SAVE[USART_SAV_LEN];//存放缓存
extern UART_HandleTypeDef huart1;
extern int target1;
extern int location;
#include <stdio.h>
#include <string.h>

char temp_str[30];    // 临时子串

void ReadStrUnit(char * str,char *temp_str,int idx,int len)  // 从母串中获取与子串长度相等的临时子串
{
    int index ;
    for(index=0; index < len; index++)
    {
        temp_str[index] = str[idx+index];
    }
    temp_str[index] = '\0';
}

int GetSubStrPos(char *str1,char *str2)
{
    int idx = 0;
    
    int len1 = strlen(str1);
    int len2 = strlen(str2);

    if( len1 < len2)
    {
        printf("error 1 \n"); // 子串比母串长
        return -1;
    }

    while(1)
    {
        ReadStrUnit(str1,temp_str,idx,len2);    // 不断获取的从 母串的 idx 位置处更新临时子串
        if(strcmp(str2,temp_str)==0)break;      // 若临时子串和子串一致，结束循环
        idx++;                                  // 改变从母串中取临时子串的位置
        if(idx>=len1)return -1;                 // 若 idx 已经超出母串长度，说明母串不包含该子串
    }

    return idx;    // 返回子串第一个字符在母串中的位置
}


void waibu()
{ int flag;
  while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_RX)//串口状态
  {
  int con=0;
  HAL_UART_Receive(&huart1,(uint8_t *)USART_RX_SAVE[con],1,1000);
  con++;
//将数据放入缓存
  
  
    
    int i = -1;
    i = GetSubStrPos(USART_RX_ORDER,USART_RX_SAVE);
    if( i<0 )
    {
        uprintf("not found\n");
    }
    else if(i==0)
    {
        target1=-1100;
    }
    else if(i==5)
    {
     location=11000;
    }
      uprintf("i = %d\n",i);
for(flag=0;flag>9;flag++)
{          USART_RX_SAVE[flag]='0';        
}//缓存清0；
  }
}
void setorder()//用于将命令加入命令集，打印出给命令代号
{uprintf("put new order!\r\n");
   int con=0;
  int flag;
  HAL_UART_Receive(&huart1,(uint8_t *)USART_RX_SAVE[con],1,1000);
  con++;
strcat(USART_RX_ORDER,USART_RX_SAVE);
USART_RX_ORDER[strlen(USART_RX_ORDER)]='/0';
  int ord = -1;
    ord = GetSubStrPos(USART_RX_ORDER,USART_RX_SAVE);
    uprintf("命令序号 %d\n",ord);
    for(flag=0;flag>9;flag++)
{          USART_RX_SAVE[flag]='0';        
}//缓存清0；
}

void help()
{


}



