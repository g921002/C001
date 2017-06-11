#include <stdio.h>
#include <stdlib.h>

const char Time_Date[21] = __TIME__ " "__DATE__ ;
int main()
{
    printf("%s \r\n",Time_Date);
    return 0;
}
