#include <stdio.h>
#include <math.h>
#include "string.h"
#include "utility.h"
void reverseString(char* a, int len)
{
	int i = 0, j= len - 1, temp;
	while(i<j)
	{
		temp = a[i];
		a[i] = a[j];
		a[j] = temp;
		i++;
		j--;
	}
}

int itoa(int n, char* a, int d)
{
	int tmpN = n;
	if(n < 0)
		tmpN *= -1;
	
	int i = 0;
	while(tmpN)
	{
		a[i++] = (tmpN%10)+'0';
		tmpN /= 10;
	}
	
	while(i<d)
		a[i++] = '0';
	
	if(n>=0)
		a[i++] = '+';
	else
		a[i++] = '-';
	
	a[i] = '\0';
	reverseString(a, i);
	return i;

	
}
int itoa2(int n, char* a, int d)
{
	int tmpN = n;
	if(n < 0)
		tmpN *= -1;
	
	int i = 0;
	while(tmpN)
	{
		a[i++] = (tmpN%10)+'0';
		tmpN /= 10;
	}
	
	while(i<d)
		a[i++] = '0';
	reverseString(a, i);
	a[i] = '\0';
	return i;
}
int itoa3(int n, char* a, int d)
{
	int tmpN = n;
	if(n < 0)
		tmpN *= -1;
	
	int i = 0;
	while(tmpN)
	{
		a[i++] = (tmpN%10)+'0';
		tmpN /= 10;
	}
	
	while(i<d)
		a[i++] = '0';
	reverseString(a, i);
	//a[i] = '\0';
	return i;
}
int intToStr(int x, char str[], int d)
{
	int i = 0;
	while(x)
	{
		str[i++] = (x%10) + '0';
		x = x/10;
	}
	
	while(i<d)
		str[i++] = '0';
	
	reverseString(str, i);
	str[i] = '\0';
	return i;
}
void ftoa(float n, char* a, int d, int afterpoint)
{
	int ipart = (int) n ;
	
	float fpart = n - (float)ipart;
	
	int i = intToStr(ipart, a, d);
	
	if(afterpoint != 0)
	{
		a[i] = '.';
		
		fpart = fpart * pow(10, afterpoint);
		//fpart = fpart * pow(10.0, (double)afterpoint) + 0.5;
		
		intToStr((int)fpart, a + i + 1, afterpoint);
	}
}