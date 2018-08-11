#include <stdio.h>

// from http://cs-fundamentals.com/tech-interview/c/c-program-to-count-number-of-ones-in-unsigned-integer.php
int countSetBits(unsigned int n) {
    n = n - ((n >> 1) & 0x55555555);
    n = (n & 0x33333333) + ((n >> 2) & 0x33333333);
    n = (n + (n >> 4)) & 0x0F0F0F0F;
    n = n + (n >> 8);
    n = n + (n >> 16);
    return n & 0x0000003F;
}

int processA() {

    int i = 0;

    FILE *o = fopen("out.bin", "ab");
    FILE *f = fopen("dump.bin", "rb");

    char buf[32];

    while ((i = fread(buf, 32, 1, f)) == 1) {
    
        int c = 0;
        int max = 32 / 4;
        int sum = 0;
        for (c = 0; c < max; c++) {
            int val = ((int *)buf)[c];
            sum += countSetBits(val);
        }

        unsigned char tmp = sum;
        fwrite(&tmp, 1, 1, o);
    }

}

int processB() {


    FILE *o = fopen("out.bin", "ab");
    FILE *f = fopen("dump.bin", "rb");

    fseek(f, 0L, SEEK_END);
    int sz = ftell(f);
    rewind(f);

    char *buf = malloc(sz);
    fread(buf, 1, sz, f);
    
    int i = 0;

    int mx = 100;

    for(i=0;i<sz-mx;i+=mx/4){
        int c = 0;
        int max = mx / 4;
        int sum = 0;

        char *b = buf+i;
        for (c = 0; c < max; c++) {
            int val = ((int *)b)[c];
            sum += countSetBits(val);
        }

        unsigned short tmp = sum*200;
        printf("sum %d\n",tmp);
        fwrite(&tmp, 2, 1, o);
    }

}


int main(){

    //processA();
    processA();
}
