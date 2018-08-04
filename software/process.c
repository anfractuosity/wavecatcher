#include <stdio.h>


// from http://cs-fundamentals.com/tech-interview/c/c-program-to-count-number-of-ones-in-unsigned-integer.php
int countSetBits(unsigned int n)
{
  n = n - ((n >> 1) & 0x55555555);
  n = (n & 0x33333333) + ((n >> 2) & 0x33333333);
  n = (n + (n >> 4)) & 0x0F0F0F0F;
  n = n + (n >> 8);
  n = n + (n >> 16);
  return n & 0x0000003F;
}


int main(){

        int i = 0;

        FILE *o = fopen("out.bin","ab");
        FILE *f = fopen("dump.bin","rb");
        
        
        char buf[32];

        printf("%d\n",fread(buf,32,1,f));

        while ((i = fread(buf,32,1,f)) == 1){
            int c = 0;
            int max = 32/4;
            int sum = 0;
            for(c=0;c<max;c++){
                int val = ((int*)buf)[c];
                sum += countSetBits(val);
            }


            printf("here\n");
            unsigned char tmp = sum;
            fwrite(&tmp,1,1,o);
        
        }

}
