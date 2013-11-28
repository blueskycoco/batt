#include "stm32f10x.h"
#include "LM096.h"

main()
{  
    int val1=0,val2=1,val3=2,val4=3,val5=4,val6=5;
    uint8_t bat1=20,bat2=0;
    u8 str[100];
	/*将数字信息填充到str里*/
    sprintf(str,"%d%d%d%d%d.%d",val1,val2,val3,val4,val5,val6);
	/*初始化ssd1306*/
    ssd1306_init();
	/*绘制缓冲区，包含电池信息和数字信息*/
    draw(bat1,bat2,str);
	/*打开显示*/
    display();
    while(1){
        val1++;
        val2++;
        val3++;
        val4++;
        val5++;			
        val6++;			
        bat1++;
        bat2++;
        sprintf(str,"%d%d%d%d%d.%d",val1,val2,val3,val4,val5,val6);
        draw(bat1,bat2,str);
        display();
        Delay(300000);
        if(val1==9)
            val1=0;
        if(val2==9)
            val2=0;
        if(val3==9)
            val3=0;
        if(val4==9)
            val4=0;
        if(val5==9)
            val5=0;
        if(val6==9)
            val6=0;
        if(bat1==100)
            bat1=0;
        if(bat2==100)
            bat2=0;
    }
}
